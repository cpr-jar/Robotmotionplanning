"""Optional real-browser editor test, using the real local service and exporter."""

from pathlib import Path
import argparse
import json
import shutil
import threading
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from create_viewer import ROOT
from editor_server import EditorServer, SimulationService
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exporter", type=Path, default=ROOT / "build-motion-viewer/motion_viewer_exporter")
    parser.add_argument("--chromium", default=shutil.which("chromium") or shutil.which("chromium-browser"))
    parser.add_argument("--screenshot", type=Path)
    options = parser.parse_args()
    if not options.chromium:
        parser.error("Set --chromium to an installed Chromium executable")
    service = SimulationService(options.exporter)
    server = EditorServer(service, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(
                executable_path=options.chromium,
                headless=True,
                args=[
                    "--no-sandbox",
                    "--disable-dev-shm-usage",
                    "--use-angle=swiftshader",
                    "--enable-unsafe-swiftshader",
                ],
            )
            page = browser.new_page(viewport={"width": 1760, "height": 1200}, device_scale_factor=1)
            errors, external = [], []
            page.on("pageerror", lambda error: errors.append(str(error)))

            def route_local(route):
                if route.request.url.startswith(server.url):
                    route.continue_()
                else:
                    external.append(route.request.url)
                    route.abort()

            page.route("**/*", route_local)
            page.goto(server.url)
            page.wait_for_function("document.body.dataset.editorState==='ready'")
            assert "models/ur5e/profile.yaml" in page.locator("#source").input_value()
            assert page.locator("#example option").count() == 5
            assert page.locator("#preview").is_hidden()
            original = page.locator("#source").input_value()
            page.locator("#filename").fill("testprogramm.yaml")
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current'", timeout=120000)
            viewer = page.frame_locator("#preview")
            viewer.locator("body[data-viewer-state=ready][data-render-mode=mesh]").wait_for(timeout=30000)
            assert viewer.locator("body").get_attribute("data-mesh-links") == "7"
            viewer.locator("#next").click()
            assert viewer.locator("body").get_attribute("data-current-tick") == "1"
            page.locator("#source").fill(original + "\n# Geänderter Entwurf\n")
            assert page.locator("body").get_attribute("data-result-state") == "stale"
            with page.expect_download() as download:
                page.locator("#save").click()
            assert Path(download.value.path()).read_text() == page.locator("#source").input_value()
            with page.expect_download() as download:
                page.locator("#download-snapshot").click()
            assert Path(download.value.path()).read_text() == original
            with page.expect_download() as download:
                page.locator("#download-html").click()
            downloaded = Path(download.value.path()).read_text()
            assert '"triangle_count":121789' in downloaded
            assert "Ludovic Delval" in downloaded
            # Invalid late motion must keep the previous preview, with line navigation.
            prior_url = page.locator("#preview").get_attribute("src")
            bad = original + "  - {type: PTP, joints: [100,0,0,0,0,0]}\n"
            page.locator("#source").fill(bad)
            page.locator("#simulate").click()
            page.locator("#diagnostic").wait_for(state="visible")
            assert "Zeile" in page.locator("#diagnostic-text").inner_text()
            assert page.locator("#preview").get_attribute("src") == prior_url
            page.locator("#go-to-error").click()
            selected = page.locator("#source").evaluate("e=>e.value.slice(e.selectionStart,e.selectionEnd)")
            assert "100" in selected
            assert "letzte gültige" in page.locator("#run-status").inner_text()
            # Reload restores the local draft rather than discarding it.
            page.wait_for_function(
                "document.getElementById('draft-note').textContent.includes('gespeichert')"
            )
            page.wait_for_timeout(300)
            page.reload()
            page.wait_for_function("document.body.dataset.editorState==='ready'")
            assert page.locator("#source").input_value() == bad
            assert "wiederhergestellt" in page.locator("#run-status").inner_text()
            # Import does not execute anything; simulation is an explicit separate action.
            imported = original.replace("seconds: 0.3", "seconds: 0.1")
            page.locator("#import-file").set_input_files(
                {"name": "importiert.yaml", "mimeType": "text/yaml", "buffer": imported.encode()}
            )
            page.wait_for_function("document.getElementById('filename').value==='importiert.yaml'")
            assert page.locator("#source").input_value() == imported
            assert page.locator("#preview").is_hidden()
            # Edit while the genuine backend response is held: an older result must
            # be labelled stale and must never replace the in-progress source.
            edited = imported + "\n# Während der Planung geändert\n"

            def hold_result(route):
                response = route.fetch(timeout=120000)
                page.locator("#source").fill(edited)
                route.fulfill(response=response)

            page.route("**/api/simulate", hold_result)
            page.locator("#simulate").click()
            page.wait_for_function(
                "document.body.dataset.editorState==='ready' && document.body.dataset.resultState==='stale'",
                timeout=120000,
            )
            assert page.locator("#source").input_value() == edited
            assert "währenddessen geändert" in page.locator("#run-status").inner_text()
            page.unroute("**/api/simulate", hold_result)
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current'", timeout=120000)
            viewer.locator("body[data-viewer-state=ready][data-render-mode=mesh]").wait_for(timeout=30000)
            viewer.locator("#scrubber").evaluate(
                "e=>{e.value=240;e.dispatchEvent(new Event('input',{bubbles:true}));}"
            )
            if options.screenshot:
                options.screenshot.parent.mkdir(parents=True, exist_ok=True)
                page.screenshot(path=str(options.screenshot), full_page=True)
            # Reproduce the 2-rad-start report through the real editor and
            # compare the matrices actually used by the browser FK along LIN.
            page.locator("#example").select_option("ur5e_lin")
            page.locator("#load-example").click()
            page.wait_for_function("document.getElementById('filename').value==='ur5e_lin_programm.yaml'")
            assert "models/ur5e/profile_pick_place_simulation.yaml" in page.locator("#source").input_value()
            page.locator("#cycle-ms").fill("10")
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current' && document.body.dataset.editorState==='ready'", timeout=120000)
            viewer.locator("body[data-viewer-state=ready][data-render-mode=mesh]").wait_for(timeout=30000)
            assert "Orientierung konstant" in viewer.locator("#segments").inner_text()
            lin_orientation = viewer.locator("#motion-data").evaluate("""element => {
              const data=JSON.parse(element.textContent), K=MotionKinematics;
              const col=Object.fromEntries(data.columns.map((key,i)=>[key,i]));
              const segment=data.program.segments.find(s=>s.operation==='LIN');
              if(segment.orientation_mode!=='keep') throw Error('Wrong LIN orientation policy');
              const line=data.frames.filter(f=>f[col.segment]===segment.index);
              const q=f=>data.robot.joint_names.map((_,i)=>f[col['q'+i]]);
              if(q(line[0])[0]<1.9) throw Error('Expected reported 2-rad start');
              const start=K.forwardKinematics(data.robot,q(line[0])).transform;
              let maximum=0;
              for(const frame of line) {
                const actual=K.forwardKinematics(data.robot,q(frame)).transform;
                for(let r=0;r<3;r++) for(let c=0;c<3;c++)
                  maximum=Math.max(maximum,Math.abs(actual[4*r+c]-start[4*r+c]));
              }
              return {samples:line.length,maximum};
            }""")
            assert lin_orientation["samples"] > 1 and lin_orientation["maximum"] < 5e-4, lin_orientation
            # Another robot uses the same planner and the skeleton fallback.
            page.locator("#example").select_option("abb_irb120")
            page.locator("#load-example").click()
            page.wait_for_function("document.getElementById('filename').value==='abb_irb120_programm.yaml'")
            page.locator("#cycle-ms").fill("20")
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current'", timeout=120000)
            viewer.locator("body[data-viewer-state=ready][data-render-mode=skeleton]").wait_for(timeout=30000)
            assert "20 ms" in page.locator("#result-summary").inner_text()
            # Hold post-processing, then use the real button and authenticated
            # HTTP cancellation endpoint. A previous immutable preview survives.
            import editor_server
            original_prepare = editor_server.prepare_data
            entered, release = threading.Event(), threading.Event()
            prior_url = page.locator("#preview").get_attribute("src")
            def paused(*args, **kwargs):
                entered.set()
                if not release.wait(10):
                    raise RuntimeError("Cancellation fixture timed out")
                return original_prepare(*args, **kwargs)
            with patch("editor_server.prepare_data", side_effect=paused):
                page.locator("#simulate").click()
                assert entered.wait(10), "Python processing did not start"
                page.locator("#cancel-simulation").click()
                page.wait_for_function("document.getElementById('run-status').textContent.includes('Abbruch angefordert')")
                release.set()
                page.wait_for_function("document.body.dataset.editorState==='ready'")
            assert "abgebrochen" in page.locator("#diagnostic-text").inner_text()
            assert page.locator("#preview").get_attribute("src") == prior_url
            assert page.locator("#cancel-simulation").is_disabled()
            page.set_viewport_size({"width": 420, "height": 850})
            page.wait_for_timeout(100)
            assert page.evaluate("document.documentElement.scrollWidth<=window.innerWidth")
            assert not errors, errors
            assert not external, external
            print(
                json.dumps(
                    {
                        "state": "ready",
                        "browser_errors": errors,
                        "lin_orientation": lin_orientation,
                        "external_requests": external,
                        "checks": [
                            "native simulation",
                            "complete UR model",
                            "viewer playback",
                            "YAML/HTML downloads",
                            "late error preserves viewer",
                            "error line",
                            "local draft restore",
                            "YAML import",
                            "edit during simulation",
                            "LIN keeps world TCP orientation after 2-rad PTP",
                            "second robot",
                            "cancel during Python processing preserves preview",
                            "20 ms grid",
                            "mobile layout",
                        ],
                    }
                )
            )
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
        service.close()


if __name__ == "__main__":
    main()
