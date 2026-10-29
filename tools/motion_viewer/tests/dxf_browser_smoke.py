"""DXF import → one YAML command → native simulation → viewer and exact bundle."""

from pathlib import Path
import argparse
import io
import json
import shutil
import sys
import threading
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from create_viewer import ROOT
from editor_server import EditorServer, SimulationService
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exporter", type=Path, default=ROOT / "build-motion-viewer/motion_viewer_exporter")
    parser.add_argument("--chromium", default=shutil.which("chromium"))
    parser.add_argument("--screenshot", type=Path)
    options = parser.parse_args()
    service = SimulationService(options.exporter)
    server = EditorServer(service, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(
                executable_path=options.chromium,
                headless=True,
                args=[
                    "--no-sandbox",
                    "--disable-dev-shm-usage",
                    "--use-angle=swiftshader",
                    "--enable-unsafe-swiftshader",
                ],
            )
            page = browser.new_page(viewport={"width": 1760, "height": 1280})
            errors = []
            external = []
            page.on("pageerror", lambda e: errors.append(str(e)))

            def local_only(route):
                if route.request.url.startswith(server.url):
                    route.continue_()
                else:
                    external.append(route.request.url)
                    route.abort()

            page.route("**/*", local_only)
            page.goto(server.url)
            page.wait_for_function("document.body.dataset.editorState==='ready'")
            # An indentless steps sequence before another top-level field catches
            # incorrect append-at-EOF/indent handling in the insertion helper.
            source = """schema_version: 1
units: SI
robot_profile: models/ur5e/profile.yaml
cycle_s: 0.01
start: {joints: [0.2,-0.6,0.7,0.3,0.8,-0.2]}
steps:
- {type: WAIT, seconds: 0.02}
points:
  home: {joints: [0.2,-0.6,0.7,0.3,0.8,-0.2]}
"""
            page.locator("#source").fill(source)
            page.locator("#filename").fill("dxf_programm.yaml")
            raw = (ROOT / "programs/dxf/rounded_path.dxf").read_bytes()
            upload = {"name": "rounded_path.dxf", "mimeType": "application/dxf", "buffer": raw}
            page.locator("#dxf-import").set_input_files(upload)
            page.wait_for_function("document.body.dataset.dxfFiles==='1'")
            page.locator("#dxf-layer").fill("PATH")
            page.locator("#insert-dxf").click()
            submitted = page.locator("#source").input_value()
            assert submitted.index("type: DXF") < submitted.index("points:")
            assert submitted.count("type: DXF") == 1
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current'", timeout=120000)
            viewer = page.frame_locator("#preview")
            viewer.locator("body[data-viewer-state=ready][data-render-mode=mesh]").wait_for(timeout=30000)
            viewer.locator("#segments button").filter(has_text="DXF · CIRC").click()
            assert "rounded_path.dxf:" in viewer.locator("#source-label").inner_text()
            assert "Layer PATH" in viewer.locator("#source-label").inner_text()
            viewer.locator("#focus-path").click()
            viewer.locator("#reset-camera").click()
            viewer.locator("#play").click()
            page.wait_for_timeout(200)
            viewer.locator("#play").click()
            with page.expect_download() as event:
                page.locator("#download-bundle").click()
            with zipfile.ZipFile(event.value.path()) as bundle:
                assert set(bundle.namelist()) == {"dxf_programm.yaml", "rounded_path.dxf"}
                assert bundle.read("rounded_path.dxf") == raw
                assert bundle.read("dxf_programm.yaml").decode() == submitted
            if options.screenshot:
                options.screenshot.parent.mkdir(parents=True, exist_ok=True)
                page.screenshot(path=str(options.screenshot), full_page=True)
            page.wait_for_timeout(300)
            page.reload()
            page.wait_for_function("document.body.dataset.editorState==='ready'")
            assert page.locator("body").get_attribute("data-dxf-files") == "1"
            assert page.locator("#source").input_value() == submitted
            page.locator("#simulate").click()
            page.wait_for_function("document.body.dataset.resultState==='current'", timeout=120000)
            previous = page.locator("#preview").get_attribute("src")
            # Replacing a CAD attachment invalidates the old simulation even when
            # the YAML text stays identical. Unsupported selected geometry fails.
            page.locator("#dxf-import").set_input_files(
                {**upload, "buffer": raw.replace(b"LWPOLYLINE", b"SPLINE")}
            )
            page.wait_for_function("document.body.dataset.resultState==='stale'")
            page.locator("#simulate").click()
            page.locator("#diagnostic").wait_for(state="visible")
            assert "DXF SPLINE" in page.locator("#diagnostic-text").inner_text()
            assert page.locator("#preview").get_attribute("src") == previous
            page.locator("#remove-dxf").click()
            page.locator("#simulate").click()
            page.wait_for_function(
                "document.getElementById('diagnostic-text').textContent.includes('attachment')"
            )
            assert page.locator("#preview").get_attribute("src") == previous
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
                        "external_requests": external,
                        "checks": [
                            "DXF upload",
                            "one inserted command",
                            "indentless steps before points",
                            "native LIN/CIRC",
                            "CAD provenance",
                            "mesh playback",
                            "path focus",
                            "exact YAML+DXF bundle",
                            "draft restore with DXF",
                            "replaced asset invalidates preview",
                            "bad geometry preserves previous preview",
                            "missing asset",
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
