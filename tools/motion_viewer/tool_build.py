"""Build only the standalone viewer exporter; shared by the two launchers."""

from pathlib import Path
import os
import subprocess
from create_viewer import TOOL_DIR


def build_exporter(build_dir, cmake="cmake"):
    build_dir = Path(build_dir).resolve()
    subprocess.run(
        [cmake, "-S", str(TOOL_DIR), "-B", str(build_dir), "-DCMAKE_BUILD_TYPE=Release"], check=True
    )
    subprocess.run(
        [cmake, "--build", str(build_dir), "--target", "motion_viewer_exporter", "--parallel", "2"],
        check=True,
    )
    return build_dir / ("motion_viewer_exporter.exe" if os.name == "nt" else "motion_viewer_exporter")
