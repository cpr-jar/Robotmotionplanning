"""Use the real exporter to check phase reporting and deterministic benchmarking."""
import copy
from pathlib import Path
import subprocess
import tempfile
import unittest

from benchmark import benchmark_program
from create_viewer import load_export, prepare_data
from test_program_editor import EXPORTER, SHORT_PROGRAM
from create_viewer import ROOT


class PreparationMetricsTests(unittest.TestCase):
    def test_native_benchmark_and_validated_optional_metrics(self):
        with tempfile.TemporaryDirectory() as folder:
            program = Path(folder) / "short.yaml"
            source = SHORT_PROGRAM.replace("models/ur5e/profile.yaml",
                                           (ROOT / "models/ur5e/profile.yaml").as_posix())
            program.write_text(source, encoding="utf-8")
            result = benchmark_program(EXPORTER.resolve(), program, 10, 2)
            self.assertEqual(result["commands"], 3)
            self.assertEqual(len(result["runs"]), 2)
            self.assertGreater(result["median"]["native_process_s"], 0)
            self.assertGreaterEqual(result["maximum_planning_s"], result["median"]["planning_s"])
            scene_file = Path(folder) / "scene.json"
            subprocess.run([str(EXPORTER.resolve()), str(program), str(scene_file)],
                           capture_output=True, check=True)
            scene = load_export(scene_file)
        self.assertEqual(prepare_data(scene, visuals="none")["preparation"], scene["preparation"])
        for key, value in (("planning_s", -1), ("simulation_s", True), ("work_units", -1),
                           ("build_configuration", [])):
            invalid = copy.deepcopy(scene)
            invalid["preparation"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                prepare_data(invalid, visuals="none")
        # Existing exports without timing metadata remain valid.
        del scene["preparation"]
        self.assertNotIn("preparation", prepare_data(scene, visuals="none"))
