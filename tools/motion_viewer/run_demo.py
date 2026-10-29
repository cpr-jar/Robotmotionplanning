"""Build the isolated visualization tool and create its offline demo."""

from pathlib import Path
import argparse
import os
import subprocess
import webbrowser
from create_viewer import TOOL_DIR, ROOT, export_and_render


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
    subprocess.run(
        [options.cmake, "-S", str(TOOL_DIR), "-B", str(options.build_dir), "-DCMAKE_BUILD_TYPE=Release"],
        check=True,
    )
    subprocess.run(
        [
            options.cmake,
            "--build",
            str(options.build_dir),
            "--target",
            "motion_viewer_exporter",
            "--parallel",
            "2",
        ],
        check=True,
    )
    suffix = ".exe" if os.name == "nt" else ""
    data = export_and_render(
        options.program,
        options.build_dir / ("motion_viewer_exporter" + suffix),
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
