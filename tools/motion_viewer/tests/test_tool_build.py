"""Configuration selection must work with single- and multi-config generators."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tool_build import build_exporter


class ToolBuildTests(unittest.TestCase):
    def test_release_configuration_and_generator_specific_binary(self):
        for multi in (False, True):
            with self.subTest(multi=multi), tempfile.TemporaryDirectory() as folder:
                build = Path(folder).resolve()
                name = "motion_viewer_exporter.exe" if os.name == "nt" else "motion_viewer_exporter"
                expected = build / ("Release" if multi else "") / name
                expected.parent.mkdir(exist_ok=True)
                expected.touch()
                # A stale root binary must not hide a multi-config build mistake.
                (build / name).touch()
                (build / "CMakeCache.txt").write_text(
                    "CMAKE_CONFIGURATION_TYPES:STRING=Debug;Release\n" if multi else
                    "CMAKE_BUILD_TYPE:STRING=Release\n", encoding="utf-8")
                with patch("tool_build.subprocess.run") as run:
                    self.assertEqual(build_exporter(build), expected)
                arguments = run.call_args_list[1].args[0]
                self.assertEqual(arguments[arguments.index("--config") + 1], "Release")

    def test_multiconfig_does_not_fall_back_to_stale_root_or_debug_binary(self):
        with tempfile.TemporaryDirectory() as folder:
            build = Path(folder)
            name = "motion_viewer_exporter.exe" if os.name == "nt" else "motion_viewer_exporter"
            (build / name).touch()
            (build / "Debug").mkdir()
            (build / "Debug" / name).touch()
            (build / "CMakeCache.txt").write_text("CMAKE_CONFIGURATION_TYPES:STRING=Debug;Release\n",
                                                 encoding="utf-8")
            with patch("tool_build.subprocess.run"), self.assertRaises(FileNotFoundError):
                build_exporter(build)
