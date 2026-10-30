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
        [cmake, "--build", str(build_dir), "--config", "Release", "--target", "motion_viewer_exporter",
         "--parallel", "2"],
        check=True,
    )
    # CMAKE_BUILD_TYPE does not select the configuration of Visual Studio or
    # Ninja Multi-Config. Never fall back to a possibly stale Debug executable.
    cache = (build_dir / "CMakeCache.txt").read_text(encoding="utf-8")
    multi_config = any(line.startswith("CMAKE_CONFIGURATION_TYPES:") and line.partition("=")[2]
                       for line in cache.splitlines())
    executable = build_dir / ("Release" if multi_config else "") / (
        "motion_viewer_exporter.exe" if os.name == "nt" else "motion_viewer_exporter")
    if not executable.is_file():
        raise FileNotFoundError("Release exporter missing after build: " + str(executable))
    return executable
