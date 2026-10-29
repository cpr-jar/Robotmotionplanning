"""Optional real-browser check. Requires Playwright and a local Chromium binary.

The document is injected into a blank test page; all network requests are blocked.
This also works in managed browsers that disallow file:// navigation.
"""

from pathlib import Path
import argparse
import json
import shutil
from playwright.sync_api import sync_playwright

TOOL_DIR = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--html", type=Path, default=TOOL_DIR / "demo/ur5e_motion.html")
    parser.add_argument("--chromium", default=shutil.which("chromium") or shutil.which("chromium-browser"))
    parser.add_argument("--screenshot", type=Path)
    options = parser.parse_args()
    if not options.chromium:
        parser.error("Set --chromium to an installed Chromium executable")
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
        page = browser.new_page(viewport={"width": 1440, "height": 1100}, device_scale_factor=1)
        requests, errors = [], []
        page.route("**/*", lambda route: (requests.append(route.request.url), route.abort()))
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.set_content(options.html.read_text(), wait_until="load")
        page.wait_for_function("document.body.dataset.viewerState !== 'loading'")
        assert page.locator("body").get_attribute("data-viewer-state") == "ready", page.locator(
            "#error"
        ).inner_text()
        assert float(page.locator("body").get_attribute("data-fk-position-error")) < 2e-8
        assert float(page.locator("body").get_attribute("data-fk-rotation-error")) < 2e-8
        assert page.locator("#joint-table tr").count() == 6
        assert page.locator("body").get_attribute("data-render-mode") == "mesh", page.locator(
            "#model-note"
        ).inner_text()
        assert page.locator("body").get_attribute("data-mesh-links") == "7"
        assert page.locator("body").get_attribute("data-mesh-triangles") == "121789"
        assert "Ludovic Delval" in page.locator("#model-licenses").text_content()
        page.locator("#render-mode").select_option("skeleton")
        assert page.locator("#mesh-scene").is_hidden()
        page.locator("#next").click()
        assert page.locator("body").get_attribute("data-current-tick") == "1"
        page.locator("#previous").click()
        page.locator("#render-mode").select_option("mesh")
        assert page.locator("#mesh-scene").is_visible()
        # Compare the matrices actually sent to Three.js with independent Three.js
        # composition. This catches a transpose, missing visual origin or Z-up error.
        mesh_report = page.evaluate(
            """() => {
          const data=JSON.parse(document.getElementById('motion-data').textContent);
          const K=MotionKinematics, T=THREE, columns=Object.fromEntries(data.columns.map((v,i)=>[v,i]));
          const canvas=document.createElement('canvas');
          const renderer=MotionMeshes.create(canvas,data.visual_model,data.frames,columns,data.program.segments);
          const pose=p=>new T.Matrix4().compose(new T.Vector3(...p.position),
            new T.Quaternion(...p.orientation.slice(1),p.orientation[0]).normalize(),new T.Vector3(1,1,1));
          const state={index:0,radius:1,distance:3,target:[0,0,0.3],pitch:0.4,yaw:-0.7};
          let cases=0,maximum=0;
          try {
            for (const index of [0,240,data.frames.length-1]) {
              const q=data.robot.joint_names.map((_,i)=>data.frames[index][columns['q'+i]]);
              for (const changed of [false,true]) {
                const robot=JSON.parse(JSON.stringify(data.robot));
                if(changed) {
                  robot.base_pose={position:[1,-2,0.7],orientation:[0.5,0.5,-0.5,0.5]};
                  robot.tool_offset={position:[0.1,-0.05,0.2],orientation:[0.5,-0.5,0.5,0.5]};
                }
                renderer.render(K.forwardKinematics(robot,q),state,400,300,true,true);
                const world=pose(robot.base_pose), expected={};let moving=0;
                for(const joint of robot.joints) {
                  world.multiply(pose(joint.origin));
                  if(joint.type!=='fixed') {
                    const axis=new T.Vector3(...joint.axis), value=q[moving++];
                    world.multiply(joint.type==='prismatic'
                      ? new T.Matrix4().makeTranslation(...axis.multiplyScalar(value).toArray())
                      : new T.Matrix4().makeRotationAxis(axis,value));
                  }
                  expected[joint.child_link]=world.clone();
                }
                for(const visual of data.visual_model.links) {
                  const matrix=expected[visual.link].multiply(pose(visual.origin)).toArray();
                  for(let i=0;i<16;i++) maximum=Math.max(maximum,Math.abs(matrix[i]-renderer.stats.linkMatrices[visual.link][i]));
                  cases++;
                }
              }
            }
          } finally {renderer.dispose();}
          return {cases,maximum};
        }"""
        )
        assert mesh_report["cases"] == 42
        assert mesh_report["maximum"] < 2e-12, mesh_report
        page.locator("#next").click()
        assert page.locator("body").get_attribute("data-current-tick") == "1"
        page.locator("#play").click()
        page.wait_for_function("Number(document.body.dataset.currentTick)>2")
        page.locator("#play").click()
        assert page.locator("#play").inner_text().startswith("▶")
        page.locator("#joint-table tr").nth(2).click()
        assert page.locator("#joint-chart-title").inner_text().startswith("J3")
        page.locator("#degrees").check()
        assert "°" in page.locator("#joint-table .unit").first.inner_text()
        page.locator("#joint-metric").select_option("jerk")
        assert "Ruck" in page.locator("#joint-chart-title").inner_text()
        page.locator("#tcp-group").select_option("angular")
        page.locator("#tcp-order").select_option("2")
        page.locator("#segments button").nth(3).click()
        assert page.locator("body").get_attribute("data-current-segment") == "4"
        page.locator("#show-axes").check()
        canvas = page.locator("#scene").bounding_box()
        page.mouse.move(canvas["x"] + canvas["width"] / 2, canvas["y"] + canvas["height"] / 2)
        page.mouse.down()
        page.mouse.move(
            canvas["x"] + canvas["width"] / 2 + 60, canvas["y"] + canvas["height"] / 2 + 20, steps=5
        )
        page.mouse.up()
        page.mouse.wheel(0, -120)
        page.locator("#reset-camera").click()
        page.eval_on_selector(
            "#scrubber", "e=>{e.value=e.max;e.dispatchEvent(new Event('input',{bubbles:true}));}"
        )
        assert "Ende im Stillstand" in page.locator("#operation-label").inner_text()
        terminal_tick = int(page.locator("body").get_attribute("data-current-tick"))
        page.locator("#play").click()
        # Software WebGL can render the first restarted frame after >100 ms.
        # Verify a return to the first tenth, not a wall-clock-dependent tick <10.
        page.wait_for_function("end=>Number(document.body.dataset.currentTick)<end/10", arg=terminal_tick)
        page.locator("#play").click()
        # Set a moving pose for the review image and choose readable velocity charts.
        page.locator("#degrees").uncheck()
        page.locator("#joint-metric").select_option("dq")
        page.locator("#tcp-group").select_option("linear")
        page.locator("#tcp-order").select_option("0")
        page.locator("#joint-table tr").first.click()
        page.eval_on_selector(
            "#scrubber", "e=>{e.value=240;e.dispatchEvent(new Event('input',{bubbles:true}));}"
        )
        if options.screenshot:
            options.screenshot.parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(options.screenshot), full_page=True)
        page.set_viewport_size({"width": 420, "height": 850})
        page.wait_for_timeout(100)
        assert page.evaluate("document.documentElement.scrollWidth<=window.innerWidth")
        assert not errors, errors
        assert not requests, requests
        # A browser without WebGL must remain usable and report the fallback.
        fallback = browser.new_page()
        fallback.route("**/*", lambda route: (requests.append(route.request.url), route.abort()))
        fallback.on("pageerror", lambda error: errors.append(str(error)))
        fallback.evaluate(
            """() => {
          const original=HTMLCanvasElement.prototype.getContext;
          HTMLCanvasElement.prototype.getContext=function(type,...args) {
            return type==='2d' ? original.call(this,type,...args) : null;
          };
        }"""
        )
        fallback.set_content(options.html.read_text(), wait_until="load")
        assert fallback.locator("body").get_attribute("data-viewer-state") == "ready"
        assert fallback.locator("body").get_attribute("data-render-mode") == "skeleton"
        assert "nicht verfügbar" in fallback.locator("#model-note").inner_text()
        fallback.locator("#next").click()
        assert fallback.locator("body").get_attribute("data-current-tick") == "1"
        fallback.close()
        page.eval_on_selector(
            "#mesh-scene", "e=>e.dispatchEvent(new Event('webglcontextlost',{cancelable:true}))"
        )
        assert page.locator("body").get_attribute("data-render-mode") == "skeleton"
        assert "unterbrochen" in page.locator("#model-note").inner_text()
        assert not errors, errors
        assert not requests, requests
        print(
            json.dumps(
                {
                    "state": "ready",
                    "browser_errors": errors,
                    "network_requests": requests,
                    "mesh_transforms": mesh_report,
                    "checks": [
                        "7 complete mesh links",
                        "121789 triangles",
                        "42 independent link matrices",
                        "mesh/skeleton switch",
                        "missing WebGL fallback",
                        "context loss fallback",
                        "FK",
                        "step",
                        "play/pause",
                        "axis selection",
                        "units",
                        "charts",
                        "segment jump",
                        "camera drag/zoom/reset",
                        "terminal frame",
                        "restart",
                        "mobile layout",
                    ],
                }
            )
        )
        browser.close()


if __name__ == "__main__":
    main()
