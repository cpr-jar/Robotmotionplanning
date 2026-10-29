"""Editor checks using the real exporter, bounded jobs and the loopback HTTP API."""

from pathlib import Path
import http.client
import json
import os
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch

from create_viewer import ROOT
from editor_server import (
    EditorError,
    EditorServer,
    MAX_REQUEST_BYTES,
    RESULT_LIMIT,
    SimulationService,
    validate_request,
)

EXPORTER = Path(os.environ.get("MOTION_VIEWER_EXPORTER", ROOT / "build-motion-viewer/motion_viewer_exporter"))
SHORT_PROGRAM = """schema_version: 1
units: SI
robot_profile: models/ur5e/profile.yaml
cycle_s: 0.01
start: {joints: [0.2,-0.6,0.7,0.3,0.8,-0.2]}
steps:
  - {type: WAIT, seconds: 0.02}
"""


def request_for(source=SHORT_PROGRAM, cycle=10):
    return {"source": source, "filename": "eigenes_programm.yaml", "cycle_ms": cycle}


class ProgramEditorTests(unittest.TestCase):
    def setUp(self):
        self.service = SimulationService(EXPORTER)

    def tearDown(self):
        self.service.close()

    def test_successful_native_run_preserves_exact_source_snapshot_and_grid(self):
        for cycle in (1, 20):
            with self.subTest(cycle=cycle):
                result = self.service.simulate(request_for(cycle=cycle))
                self.assertEqual(result["cycle_ms"], cycle)
                self.assertEqual(result["segments"], 1)
                self.assertEqual(result["commands"], round(result["duration_s"] / (cycle * 0.001)) + 1)
                self.assertTrue(result["full_model"])
                source, filename = self.service.result_file(result["id"], source=True)
                self.assertEqual(source.decode(), SHORT_PROGRAM)
                self.assertEqual(filename, "eigenes_programm.yaml")
                html, _ = self.service.result_file(result["id"])
                self.assertIn(b"MotionMeshes", html)
                self.assertIn(b'"triangle_count":121789', html)
                self.assertFalse((self.service.folder / result["id"] / "scene.json").exists())

    def test_late_error_has_line_and_step_and_preserves_previous_valid_result(self):
        result = self.service.simulate(request_for())
        prior = self.service.result_file(result["id"])[0]
        source = SHORT_PROGRAM + "  - {type: PTP, joints: [100,0,0,0,0,0]}\n"
        with self.assertRaises(EditorError) as caught:
            self.service.simulate(request_for(source))
        detail = caught.exception.diagnostic
        self.assertEqual(caught.exception.status, 422)
        self.assertEqual(detail["line"], 8)
        self.assertEqual(detail["step"], 2)
        self.assertEqual(detail["filename"], "eigenes_programm.yaml")
        self.assertNotIn(str(self.service.folder), detail["message"])
        self.assertEqual(self.service.result_file(result["id"])[0], prior)
        self.assertEqual(len(list(self.service.folder.iterdir())), 1)

    def test_parser_errors_and_model_path_escape_are_rejected(self):
        for profile in ("../models/ur5e/profile.yaml", "/etc/passwd", "models/missing/profile.yaml"):
            with self.subTest(profile=profile), self.assertRaises(EditorError) as caught:
                self.service.simulate(request_for(SHORT_PROGRAM.replace("models/ur5e/profile.yaml", profile)))
            self.assertEqual(caught.exception.diagnostic["line"], 3)
        source = SHORT_PROGRAM.replace("steps:", "robot_urdf: /etc/passwd\nsteps:")
        with self.assertRaises(EditorError):
            self.service.simulate(request_for(source))
        with self.assertRaises(EditorError) as caught:
            self.service.simulate(request_for(SHORT_PROGRAM + "units: SI\n"))
        self.assertIn("Duplicate", caught.exception.diagnostic["message"])
        self.assertFalse(list(self.service.folder.iterdir()))

    def test_symlink_cannot_escape_model_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "models").mkdir()
            (root / "models/outside").symlink_to(ROOT / "models/ur5e", target_is_directory=True)
            self.service.workspace = root
            with self.assertRaises(EditorError) as caught:
                self.service.simulate(request_for(SHORT_PROGRAM.replace("models/ur5e/", "models/outside/")))
            self.assertIn("inside models/", caught.exception.diagnostic["message"])

    def test_request_validation_budgets_and_unicode(self):
        invalid = [
            {},
            request_for(""),
            request_for("\ud800"),
            request_for("x" * (1024 * 1024 + 1)),
            {**request_for(), "filename": "../program.yaml"},
            request_for(cycle=True),
            request_for(cycle=0),
            request_for(cycle=21),
            request_for(cycle=1.5),
        ]
        for value in invalid:
            with self.subTest(value=str(value)[:70]), self.assertRaises(EditorError):
                validate_request(value)
        self.assertEqual(validate_request(request_for())[0], SHORT_PROGRAM)

    def test_busy_and_timeout_never_publish_partial_results(self):
        self.service._simulation_lock.acquire()
        try:
            with self.assertRaises(EditorError) as caught:
                self.service.simulate(request_for())
            self.assertEqual(caught.exception.status, 409)
        finally:
            self.service._simulation_lock.release()
        with patch("editor_server.subprocess.run", side_effect=subprocess.TimeoutExpired("exporter", 120)):
            with self.assertRaisesRegex(EditorError, "Zeitbudget"):
                self.service.simulate(request_for())
        self.assertFalse(list(self.service.folder.iterdir()))
        # The failed job must release its slot so a later genuine run succeeds.
        self.assertTrue(self.service.simulate(request_for())["commands"] > 0)

    def test_result_retention_is_bounded_and_old_urls_expire(self):
        results = [self.service.simulate(request_for()) for _ in range(RESULT_LIMIT + 1)]
        self.assertEqual(len(list(self.service.folder.iterdir())), RESULT_LIMIT)
        with self.assertRaises(EditorError) as caught:
            self.service.result_file(results[0]["id"])
        self.assertEqual(caught.exception.status, 404)
        self.assertTrue(self.service.result_file(results[-1]["id"])[0])

    def test_examples_have_portable_paths_and_work_with_existing_native_parser(self):
        for name in ("ur5e", "abb_irb120", "fanuc_lrmate200id"):
            example = self.service.example(name)
            self.assertIn(f"robot_profile: models/{name}/profile.yaml", example["source"])
            if name != "ur5e":  # The longer UR visual demo is covered by existing tool tests/browser.
                result = self.service.simulate(
                    {"source": example["source"], "filename": example["filename"], "cycle_ms": 10}
                )
                self.assertGreater(result["segments"], 3)
                self.assertFalse(result["full_model"])
        with self.assertRaises(EditorError):
            self.service.example("../../etc/passwd")


class EditorHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.service = SimulationService(EXPORTER)
        cls.server = EditorServer(cls.service, 0)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5)
        cls.service.close()

    def call(self, method, path, body=None, headers=None):
        client = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=30)
        try:
            client.request(method, path, body, headers or {})
            response = client.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            client.close()

    def test_real_http_round_trip_and_immutable_downloads(self):
        status, headers, body = self.call("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"Programmeditor", body)
        self.assertEqual(headers["Cache-Control"], "no-store")
        status, _, body = self.call("GET", "/api/config")
        token = json.loads(body)["token"]
        status, _, body = self.call(
            "POST",
            "/api/simulate",
            json.dumps(request_for()),
            {
                "Content-Type": "application/json",
                "X-Motion-Token": token,
                "Origin": self.server.url.rstrip("/"),
            },
        )
        self.assertEqual(status, 200, body)
        result = json.loads(body)
        status, _, html = self.call("GET", result["preview_url"])
        self.assertEqual(status, 200)
        self.assertIn(b"MotionKinematics", html)
        status, headers, source = self.call("GET", result["source_url"])
        self.assertEqual(source.decode(), SHORT_PROGRAM)
        self.assertIn('filename="eigenes_programm.yaml"', headers["Content-Disposition"])
        status, headers, downloaded = self.call("GET", result["html_url"])
        self.assertEqual(downloaded, html)
        self.assertIn("attachment", headers["Content-Disposition"])

    def test_http_boundaries_host_origin_token_body_and_static_routes(self):
        headers = {"Content-Type": "application/json", "X-Motion-Token": self.server.token}
        for changed, expected in [
            ({"Host": "other.example"}, 403),
            ({"Origin": "https://other.example"}, 403),
            ({"X-Motion-Token": "wrong"}, 403),
            ({"X-Motion-Token": "é"}, 403),
            ({"Content-Type": "text/plain"}, 415),
            ({"Content-Length": str(MAX_REQUEST_BYTES + 1)}, 413),
        ]:
            with self.subTest(headers=changed):
                status, _, _ = self.call("POST", "/api/simulate", "{}", {**headers, **changed})
                self.assertEqual(status, expected)
        for body in (
            '{"source":1,"source":2}',
            '{"source":NaN}',
            '{"source":"\\ud800","filename":"a.yaml","cycle_ms":10}',
        ):
            status, _, _ = self.call("POST", "/api/simulate", body, headers)
            self.assertEqual(status, 400)
        for path in ("/../README.md", "/static/../../README.md", "/results/nope/viewer.html"):
            self.assertEqual(self.call("GET", path)[0], 404)
        self.assertEqual(
            self.call("GET", "http://[", headers={"Host": "127.0.0.1:" + str(self.server.server_port)})[0],
            404,
        )
        self.assertEqual(self.call("GET", "/", headers={"Host": "other.example"})[0], 403)


if __name__ == "__main__":
    unittest.main()
