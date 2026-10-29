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
    parser.add_argument('--html', type=Path, default=TOOL_DIR / 'demo/ur5e_motion.html')
    parser.add_argument('--chromium', default=shutil.which('chromium') or shutil.which('chromium-browser'))
    parser.add_argument('--screenshot', type=Path)
    options = parser.parse_args()
    if not options.chromium: parser.error('Set --chromium to an installed Chromium executable')
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=options.chromium, headless=True,
                                             args=['--no-sandbox', '--disable-dev-shm-usage'])
        page = browser.new_page(viewport={'width':1440, 'height':1100}, device_scale_factor=1)
        requests, errors = [], []
        page.route('**/*', lambda route: (requests.append(route.request.url), route.abort()))
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.set_content(options.html.read_text(), wait_until='load')
        page.wait_for_function("document.body.dataset.viewerState !== 'loading'")
        assert page.locator('body').get_attribute('data-viewer-state') == 'ready', page.locator('#error').inner_text()
        assert float(page.locator('body').get_attribute('data-fk-position-error')) < 2e-8
        assert float(page.locator('body').get_attribute('data-fk-rotation-error')) < 2e-8
        assert page.locator('#joint-table tr').count() == 6
        page.locator('#next').click()
        assert page.locator('body').get_attribute('data-current-tick') == '1'
        page.locator('#play').click()
        page.wait_for_function("Number(document.body.dataset.currentTick)>2")
        page.locator('#play').click()
        assert page.locator('#play').inner_text().startswith('▶')
        page.locator('#joint-table tr').nth(2).click()
        assert page.locator('#joint-chart-title').inner_text().startswith('J3')
        page.locator('#degrees').check()
        assert '°' in page.locator('#joint-table .unit').first.inner_text()
        page.locator('#joint-metric').select_option('jerk')
        assert 'Ruck' in page.locator('#joint-chart-title').inner_text()
        page.locator('#tcp-group').select_option('angular')
        page.locator('#tcp-order').select_option('2')
        page.locator('#segments button').nth(3).click()
        assert page.locator('body').get_attribute('data-current-segment') == '4'
        page.locator('#show-axes').check()
        canvas = page.locator('#scene').bounding_box()
        page.mouse.move(canvas['x']+canvas['width']/2, canvas['y']+canvas['height']/2)
        page.mouse.down(); page.mouse.move(canvas['x']+canvas['width']/2+60,canvas['y']+canvas['height']/2+20,steps=5);page.mouse.up()
        page.mouse.wheel(0,-120)
        page.locator('#reset-camera').click()
        page.eval_on_selector('#scrubber', "e=>{e.value=e.max;e.dispatchEvent(new Event('input',{bubbles:true}));}")
        assert 'Ende im Stillstand' in page.locator('#operation-label').inner_text()
        page.locator('#play').click()
        page.wait_for_function("Number(document.body.dataset.currentTick)<10")
        page.locator('#play').click()
        # Set a moving pose for the review image and choose readable velocity charts.
        page.locator('#degrees').uncheck()
        page.locator('#joint-metric').select_option('dq')
        page.locator('#tcp-group').select_option('linear')
        page.locator('#tcp-order').select_option('0')
        page.locator('#joint-table tr').first.click()
        page.eval_on_selector('#scrubber', "e=>{e.value=240;e.dispatchEvent(new Event('input',{bubbles:true}));}")
        if options.screenshot:
            options.screenshot.parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(options.screenshot), full_page=True)
        page.set_viewport_size({'width':420,'height':850})
        page.wait_for_timeout(100)
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
        assert not errors, errors
        assert not requests, requests
        print(json.dumps({'state':'ready','browser_errors':errors,'network_requests':requests,
                          'checks':['FK','step','play/pause','axis selection','units','charts','segment jump',
                                    'camera drag/zoom/reset','terminal frame','restart','mobile layout']}))
        browser.close()


if __name__ == '__main__':
    main()
