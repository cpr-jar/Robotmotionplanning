"""Run the three real-browser regression scripts with one Chromium installation."""
from pathlib import Path
import argparse
import subprocess
import sys
from playwright.sync_api import sync_playwright

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--exporter", type=Path, required=True)
parser.add_argument("--chromium", type=Path)
options = parser.parse_args()
if options.chromium:
    chromium = options.chromium
else:
    with sync_playwright() as playwright:
        chromium = Path(playwright.chromium.executable_path)
        # Complete driver initialization before closing this discovery context.
        probe = playwright.request.new_context()
        probe.dispose()
folder = Path(__file__).resolve().parent
for script in ("browser_smoke.py", "editor_browser_smoke.py", "dxf_browser_smoke.py"):
    command = [sys.executable, str(folder / script), "--chromium", str(chromium)]
    if script != "browser_smoke.py":
        command += ["--exporter", str(options.exporter.resolve())]
    subprocess.run(command, check=True, timeout=300)
