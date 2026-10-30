"""Real worker and postprocessing cancellation, commit races and bounded display storage."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import csv
import io
import os
import tempfile
import threading
import time
import tracemalloc
import unittest
from unittest.mock import patch

import create_viewer as viewer
from editor_server import EditorError, SimulationService
from job_budget import JobBudget, JobStopped
from test_program_editor import EXPORTER, SHORT_PROGRAM, request_for


class JobBudgetTests(unittest.TestCase):
    def test_lazy_lines_preserve_csv_newline_conventions(self):
        for ending in ("\n", "\r\n", "\r"):
            text = ending.join(['a,b', '1,2', '3,4']) + ending
            self.assertEqual(list(csv.reader(viewer.trace_lines(text))), [['a', 'b'], ['1', '2'], ['3', '4']])
        self.assertEqual(list(csv.reader(viewer.trace_lines('a,"b\nc"\n'))), [['a', 'b\nc']])

    def test_deadline_and_thread_safe_cancellation(self):
        with self.assertRaises(ValueError):
            JobBudget(float("nan"))
        budget = JobBudget(1)
        budget.deadline = 0
        with self.assertRaises(JobStopped) as caught:
            budget.checkpoint()
        self.assertEqual(caught.exception.code, "deadline_exceeded")
        budget = JobBudget(1)
        thread = threading.Thread(target=budget.cancelled.set)
        thread.start()
        thread.join()
        with self.assertRaises(JobStopped) as caught:
            budget.remaining()
        self.assertEqual(caught.exception.code, "cancelled")

    def test_actual_worker_cancel_and_timeout_reap_and_remove_partial_files(self):
        if os.name == "nt":
            self.skipTest("POSIX worker fixture")
        with tempfile.TemporaryDirectory() as folder:
            script = Path(folder) / "worker"
            marker = Path(folder) / "started"
            script.write_text("#!/usr/bin/env python3\nfrom pathlib import Path\nimport time\nPath(" + repr(str(marker)) + ").write_text('started')\ntime.sleep(30)\n")
            script.chmod(0o700)
            for timeout in (10, 0.15):
                service = SimulationService(script, timeout_s=timeout)
                try:
                    with ThreadPoolExecutor(max_workers=1) as executor:
                        job_id = "a" * 32
                        future = executor.submit(service.simulate, request_for() | {"job_id": job_id})
                        if timeout == 10:
                            for _ in range(200):
                                if marker.exists():
                                    break
                                time.sleep(0.01)
                            self.assertTrue(marker.exists(), "Worker did not start")
                            self.assertFalse(service.cancel("b" * 32)["cancelled"])
                            self.assertTrue(service.cancel(job_id)["cancelled"])
                        with self.assertRaises(EditorError) as caught:
                            future.result(timeout=4)
                        self.assertEqual(caught.exception.diagnostic["code"], "cancelled" if timeout == 10 else "deadline_exceeded")
                    self.assertFalse(list(service.folder.iterdir()))
                    self.assertFalse(service.cancel(job_id)["cancelled"])
                finally:
                    service.close()
                marker.unlink(missing_ok=True)

    def test_cancel_during_python_phase_preserves_prior_and_releases_slot(self):
        service = SimulationService(EXPORTER)
        try:
            prior = service.simulate(request_for())
            snapshot = service.result_file(prior["id"])[0]
            entered, resume = threading.Event(), threading.Event()
            original = viewer.prepare_data
            def paused(*args, **kwargs):
                entered.set()
                if not resume.wait(5):
                    raise RuntimeError("Fixture resume timeout")
                return original(*args, **kwargs)
            with patch("editor_server.prepare_data", side_effect=paused), ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(service.simulate, request_for() | {"job_id": "c" * 32})
                self.assertTrue(entered.wait(5))
                self.assertTrue(service.cancel("c" * 32)["cancelled"])
                resume.set()
                with self.assertRaises(EditorError) as caught:
                    future.result(timeout=10)
                self.assertEqual(caught.exception.diagnostic["code"], "cancelled")
            self.assertEqual(service.result_file(prior["id"])[0], snapshot)
            self.assertEqual(len(service._results), 1)
            self.assertTrue(service.simulate(request_for())["commands"])
        finally:
            service.close()

    def test_deadline_after_render_prevents_commit(self):
        service = SimulationService(EXPORTER)
        try:
            from editor_server import render_html
            def expire(data):
                html = render_html(data)
                with service._jobs_lock:
                    service._active_job[1].deadline = 0
                return html
            with patch("editor_server.render_html", side_effect=expire):
                with self.assertRaises(EditorError) as caught:
                    service.simulate(request_for())
                self.assertEqual(caught.exception.diagnostic["code"], "deadline_exceeded")
            self.assertFalse(service._results)
            self.assertFalse(list(service.folder.iterdir()))
        finally:
            service.close()

    def test_streaming_validation_has_bounded_numeric_storage_and_checks_hidden_rows(self):
        # Capture a tiny real WAIT export, then build a stationary 100k-row trace.
        import subprocess
        with tempfile.TemporaryDirectory() as folder:
            source, exported = Path(folder)/"wait.yaml", Path(folder)/"wait.json"
            source.write_text(SHORT_PROGRAM.replace("models/ur5e/profile.yaml", str(viewer.ROOT/"models/ur5e/profile.yaml")))
            subprocess.run([str(EXPORTER),str(source),str(exported)],check=True,stdout=subprocess.DEVNULL)
            data = viewer.load_export(exported)
        rows = list(csv.reader(io.StringIO(data["trace_csv"])))
        columns = {name:i for i,name in enumerate(rows[0])}
        count = 100000
        out = io.StringIO(newline="")
        writer = csv.writer(out,lineterminator="\n")
        writer.writerow(rows[0])
        row = rows[1]
        for tick in range(count):
            row[columns["tick"]] = str(tick)
            row[columns["time_s"]] = row[columns["timestamp_s"]] = str(tick * .01)
            writer.writerow(row)
        data["trace_csv"] = out.getvalue()
        data["program"].update(total_ticks=count-1, commands=count, duration_s=(count-1)*.01)
        data["program"]["segments"][0]["end_s"] = (count-1)*.01
        out.close()
        tracemalloc.start()
        reduced = viewer.prepare_data(data, max_frames=100, visuals="none")
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        # Both passes reuse the input string. Only <=100 numeric rows remain;
        # neither all floats nor StringIO's 4-byte Unicode copy may be retained.
        self.assertLess(peak, 4*1024*1024)
        self.assertLessEqual(len(reduced["frames"]),100)
        self.assertEqual(reduced["original_frame_count"],count)
        # A violation at a decimated tick must still be reported.
        lines = data["trace_csv"].splitlines()
        bad = lines[2].split(',')
        bad[columns['dq0']] = '100000'
        lines[2] = ','.join(bad)
        data["trace_csv"] = '\n'.join(lines)+'\n'
        reduced = viewer.prepare_data(data, max_frames=100, visuals="none")
        self.assertTrue(reduced["sample_limit_violations"])
        self.assertFalse(any(frame[columns['tick']] == 1 for frame in reduced['frames']))
        print(f"Streaming 100000 rows: CSV={len(data['trace_csv'])} bytes, traced peak={peak} bytes, display={len(reduced['frames'])}")
