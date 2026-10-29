"""Build the isolated visualization tool and create its offline demo."""

from pathlib import Path
import argparse
import webbrowser
from create_viewer import TOOL_DIR, ROOT, export_and_render
from tool_build import build_exporter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--program", type=Path, default=TOOL_DIR / "demo/ur5e_visual_demo.yaml")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build-motion-viewer")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--cycle-ms", type=int, choices=range(1, 21), default=10)
    parser.add_argument("--visuals", choices=["auto", "none", "ur5e"], default="auto")
    parser.add_argument(
        "--open", action="store_true", help="Open generated HTML in the local default browser"
    )
    options = parser.parse_args()
    options.output = options.output or options.build_dir / "motion.html"
    if options.output.suffix.lower() != ".html":
        parser.error("Output needs .html extension")
    exporter = build_exporter(options.build_dir, options.cmake)
    data = export_and_render(
        options.program,
        exporter,
        options.output,
        options.cycle_ms,
        visuals=options.visuals,
    )
    print(
        f'Open {options.output.resolve()}\n{data["display_frame_count"]} visible / {data["original_frame_count"]} simulated samples'
    )
    if options.open:
        webbrowser.open(options.output.resolve().as_uri())


if __name__ == "__main__":
    main()
