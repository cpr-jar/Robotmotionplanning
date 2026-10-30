"""Loopback-only program editor backend; the existing C++ planner does the work.

Only Python's standard library is used. Simulations are serialized and published
as immutable results only after complete planning, simulation and trace checks.
"""

from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
import logging
import os
import re
import secrets
import shutil
import subprocess
import tempfile
import threading
import zipfile

from job_budget import JobBudget, JobStopped

from create_viewer import (
    ROOT,
    TOOL_DIR,
    atomic_write,
    load_export,
    prepare_data,
    render_html,
    unique_object,
    reject_constant,
)

MAX_REQUEST_BYTES = 12 * 1024 * 1024
MAX_SOURCE_BYTES = 1024 * 1024
MAX_ASSET_BYTES = 1024 * 1024
MAX_ASSETS = 4
MAX_DIAGNOSTIC_BYTES = 16384
RESULT_LIMIT = 3
EXAMPLES = {
    "ur5e": ("UR5e · PTP / LIN / Vollkreis / WAIT", TOOL_DIR / "demo/ur5e_visual_demo.yaml", "ur5e/profile.yaml"),
    "ur5e_pick_place": (
        "UR5e · Pick and Place",
        ROOT / "programs/examples/ur5e_pick_place.yaml",
        "ur5e/profile_pick_place_simulation.yaml",
    ),
    "ur5e_dxf": ("UR5e · DXF-Kontur abfahren", ROOT / "programs/examples/ur5e_dxf_demo.yaml", "ur5e/profile.yaml"),
    "abb_irb120": (
        "ABB IRB120 · Unterprogramm",
        ROOT / "programs/examples/abb_irb120_demo.yaml",
        "abb_irb120/profile.yaml",
    ),
    "fanuc_lrmate200id": (
        "FANUC LR Mate · Unterprogramm",
        ROOT / "programs/examples/fanuc_lrmate200id_demo.yaml",
        "fanuc_lrmate200id/profile.yaml",
    ),
}


class EditorError(Exception):
    def __init__(self, status, message, **location):
        super().__init__(message)
        self.status = status
        self.diagnostic = {"message": message, **location}


def validate_request(value):
    required = {"source", "filename", "cycle_ms"}
    if not isinstance(value, dict) or not required.issubset(value) or set(value) - required - {"assets", "job_id"}:
        raise EditorError(400, "Erwartet werden source, filename, cycle_ms und optional assets.")
    if "job_id" in value and (not isinstance(value["job_id"], str) or not re.fullmatch(r"[a-f0-9]{32}", value["job_id"])):
        raise EditorError(400, "Ungültige Auftragskennung.")
    source, filename, cycle = value["source"], value["filename"], value["cycle_ms"]
    try:
        valid_source = (
            isinstance(source, str)
            and source.strip()
            and "\0" not in source
            and len(source.encode("utf-8")) <= MAX_SOURCE_BYTES
        )
    except UnicodeError:
        valid_source = False
    if not valid_source:
        raise EditorError(400, "Das Programm muss nichtleerer UTF-8-Text mit höchstens 1 MiB sein.")
    if not isinstance(filename, str) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,74}\.ya?ml", filename):
        raise EditorError(
            400, "Dateiname: maximal 80 Zeichen, Buchstaben/Ziffern/._-, Endung .yaml oder .yml."
        )
    if type(cycle) is not int or not 1 <= cycle <= 20:
        raise EditorError(400, "Das Sollwertraster muss eine ganze Zahl von 1 bis 20 ms sein.")
    assets = value.get("assets", [])
    if not isinstance(assets, list) or len(assets) > MAX_ASSETS:
        raise EditorError(400, "Höchstens vier DXF-Dateien pro Programm.")
    names = set()
    for asset in assets:
        if not isinstance(asset, dict) or set(asset) != {"filename", "content"}:
            raise EditorError(400, "DXF-Anhang benötigt filename und content.")
        name, content = asset["filename"], asset["content"]
        if not isinstance(name, str) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,75}\.dxf", name, re.I):
            raise EditorError(400, "DXF-Dateiname: maximal 80 Zeichen, ohne Verzeichnisse, Endung .dxf.")
        if name.casefold() in names:
            raise EditorError(400, "Doppelte DXF-Dateinamen sind nicht zulässig.")
        names.add(name.casefold())
        try:
            valid = (
                isinstance(content, str)
                and content.strip()
                and "\0" not in content
                and len(content.encode("utf-8")) <= MAX_ASSET_BYTES
            )
        except UnicodeError:
            valid = False
        if not valid:
            raise EditorError(400, "DXF-Anhänge müssen UTF-8-Text mit höchstens 1 MiB pro Datei sein.")
    return source, filename, cycle, assets


class SimulationService:
    def __init__(self, exporter, workspace=ROOT, timeout_s=120):
        self.exporter = Path(exporter).resolve()
        if not self.exporter.is_file():
            raise ValueError("Exporter fehlt; zuerst das separate Viewer-Werkzeug bauen.")
        self.workspace = Path(workspace).resolve()
        JobBudget(timeout_s)  # Validate configuration before starting any worker.
        self.timeout_s = timeout_s
        self._temporary = tempfile.TemporaryDirectory(prefix="motion-editor-")
        self.folder = Path(self._temporary.name)
        self._simulation_lock = threading.Lock()
        self._results_lock = threading.Lock()
        self._jobs_lock = threading.Lock()
        self._active_job = None
        self._closing = False
        self._results = OrderedDict()
        self._expired_results = set()

    def _cleanup_expired_results(self):
        # Called with the simulation lock held, after the publication commit.
        # IO cleanup must never turn an already visible result into an error.
        for expired in tuple(self._expired_results):
            try:
                shutil.rmtree(expired)
            except FileNotFoundError:
                pass
            except OSError:
                logging.getLogger(__name__).warning("Expired simulation cleanup deferred until next run.")
                continue
            self._expired_results.remove(expired)

    def cancel(self, job_id):
        if not isinstance(job_id, str) or not re.fullmatch(r"[a-f0-9]{32}", job_id):
            raise EditorError(400, "Ungültige Auftragskennung.")
        with self._jobs_lock:
            if self._active_job is None or self._active_job[0] != job_id:
                return {"job_id": job_id, "cancelled": False}
            self._active_job[1].cancelled.set()
            return {"job_id": job_id, "cancelled": True}

    def close(self):
        with self._jobs_lock:
            self._closing = True
            if self._active_job:
                self._active_job[1].cancelled.set()
        # Wait for cancellation acknowledgement before removing a worker's files.
        with self._simulation_lock:
            self._temporary.cleanup()

    def example(self, name):
        if name not in EXAMPLES:
            raise EditorError(404, "Unbekanntes Beispiel.")
        label, path, profile = EXAMPLES[name]
        source = path.read_text(encoding="utf-8")
        # Known checked-in examples use paths relative to their original folder.
        # Replace only that exact line; preserve all program lines and comments.
        profile_line = next(line for line in source.splitlines() if line.startswith("robot_profile:"))
        source = source.replace(profile_line, f"robot_profile: models/{profile}", 1)
        assets = []
        if name == "ur5e_dxf":
            source = source.replace("file: ../dxf/rounded_path.dxf", "file: rounded_path.dxf", 1)
            assets = [
                {
                    "filename": "rounded_path.dxf",
                    "content": (ROOT / "programs/dxf/rounded_path.dxf").read_text(encoding="utf-8"),
                }
            ]
        return {"label": label, "filename": name + "_programm.yaml", "source": source, "assets": assets}

    def simulate(self, request):
        source, filename, cycle, assets = validate_request(request)
        if not self._simulation_lock.acquire(blocking=False):
            raise EditorError(409, "Eine Simulation läuft bereits. Bitte deren Abschluss abwarten.")
        run = None
        published = False
        budget = JobBudget(self.timeout_s)
        job_id = request.get("job_id", secrets.token_hex(16))
        with self._jobs_lock:
            if self._closing:
                self._simulation_lock.release()
                raise EditorError(503, "Der Editor wird beendet.")
            self._active_job = (job_id, budget)
        try:
            result_id = secrets.token_hex(16)
            run = self.folder / result_id
            run.mkdir()
            program = run / filename
            program.write_bytes(source.encode("utf-8"))
            for asset in assets:
                (run / asset["filename"]).write_bytes(asset["content"].encode("utf-8"))
            exported = run / "scene.json"
            with tempfile.TemporaryFile() as diagnostics:
                budget.checkpoint()
                environment = os.environ.copy()
                environment["MOTION_PREPARATION_TIMEOUT_MS"] = str(max(1, int(budget.remaining() * 1000)))
                with subprocess.Popen(
                    [str(self.exporter), str(program), str(exported), str(cycle), "99999",
                     "--workspace", str(self.workspace)],
                    stdout=subprocess.DEVNULL, stderr=diagnostics, cwd=self.workspace, env=environment,
                ) as process:
                    try:
                        while True:
                            budget.checkpoint()
                            try:
                                process.wait(timeout=min(0.05, budget.remaining()))
                                break
                            except subprocess.TimeoutExpired:
                                continue
                        budget.checkpoint()
                    except BaseException:
                        # Kill only this job's direct native worker and reap it.
                        process.terminate()
                        try:
                            process.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                        raise
                if process.returncode:
                    diagnostics.seek(0)
                    raw = diagnostics.read(MAX_DIAGNOSTIC_BYTES).decode("utf-8", errors="replace")
                    try:
                        detail = json.loads(raw)
                        if not isinstance(detail, dict) or not isinstance(detail.get("message"), str):
                            raise ValueError("Invalid diagnostic")
                    except (ValueError, TypeError):
                        detail = {"message": raw.strip() or "Die native Simulation wurde abgelehnt."}
                    # No temporary host paths in user-facing diagnostics.
                    detail["message"] = detail["message"].replace(str(run), "<Programm>")
                    raise EditorError(422, **detail)
            data = prepare_data(load_export(exported, budget=budget), budget=budget)
            if data["sample_limit_violations"]:
                raise EditorError(
                    422,
                    "Die vollständige Simulationsspur verletzt Profilgrenzen; keine neue Ansicht freigegeben.",
                )
            budget.checkpoint()
            html = render_html(data)
            budget.checkpoint()
            atomic_write(run / "viewer.html", html)
            budget.checkpoint()
            with zipfile.ZipFile(run / "program.zip", "w", compression=zipfile.ZIP_DEFLATED) as bundle:
                bundle.write(program, filename)
                for asset in assets:
                    bundle.write(run / asset["filename"], asset["filename"])
            exported.unlink()  # Keep just the source snapshot and the finished viewer.
            summary = {
                "id": result_id,
                "filename": filename,
                "robot": data["robot"]["name"],
                "cycle_ms": cycle,
                "duration_s": data["program"]["duration_s"],
                "commands": data["program"]["commands"],
                "segments": len(data["program"]["segments"]),
                "display_frames": data["display_frame_count"],
                "full_model": "visual_model" in data,
                "preview_url": f"/results/{result_id}/viewer.html",
                "html_url": f"/downloads/{result_id}/viewer.html",
                "source_url": f"/downloads/{result_id}/program.yaml",
                "bundle_url": f"/downloads/{result_id}/program.zip",
                "dxf_files": len(assets),
            }
            # Linearize cancellation and publication: a confirmed cancellation
            # cannot race a commit; a late cancel cannot affect a later job.
            with self._jobs_lock:
                budget.checkpoint()
                with self._results_lock:
                    self._results[result_id] = (run, filename)
                    published = True
                    while len(self._results) > RESULT_LIMIT:
                        _, (expired, _) = self._results.popitem(last=False)
                        self._expired_results.add(expired)
                self._active_job = None
            self._cleanup_expired_results()
            return summary
        except EditorError:
            raise
        except JobStopped as error:
            raise EditorError(422, str(error), code=error.code) from error
        except (ValueError, KeyError, TypeError, OSError, subprocess.SubprocessError) as error:
            raise EditorError(
                422, "Keine Simulation veröffentlicht: " + str(error).replace(str(self.folder), "<Sitzung>")
            ) from error
        finally:
            with self._jobs_lock:
                if self._active_job and self._active_job[0] == job_id:
                    self._active_job = None
            if run is not None and not published:
                shutil.rmtree(run, ignore_errors=True)
            self._simulation_lock.release()

    def result_file(self, result_id, source=False, bundle=False):
        with self._results_lock:
            if result_id not in self._results:
                raise EditorError(
                    404,
                    "Simulation nicht vorhanden oder abgelaufen. Erneut simulieren; die letzten drei gültigen Stände bleiben verfügbar.",
                )
            run, filename = self._results[result_id]
            return (
                run / ("program.zip" if bundle else filename if source else "viewer.html")
            ).read_bytes(), filename


class EditorServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, service, port=8765):
        self.service = service
        self.token = secrets.token_urlsafe(32)
        super().__init__(("127.0.0.1", port), EditorHandler)
        self.hosts = {f"127.0.0.1:{self.server_port}", f"localhost:{self.server_port}"}
        self.url = f"http://127.0.0.1:{self.server_port}/"


class EditorHandler(BaseHTTPRequestHandler):
    server_version = "MotionEditor/0.5.0"

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, format, *args):
        # Programs and tokens are never logged. The launcher prints the URL.
        pass

    def respond(self, status, body, mime="application/json; charset=utf-8", attachment=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=True, allow_nan=False).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "SAMEORIGIN")
        if attachment:
            self.send_header("Content-Disposition", f'attachment; filename="{attachment}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def check_host(self):
        if self.headers.get("Host") not in self.server.hosts:
            raise EditorError(403, "Dieser Dienst ist nur über seine lokale Adresse erreichbar.")

    def do_GET(self):
        try:
            self.check_host()
            # Fixed local routes need no URL authority parsing or path decoding.
            path = self.path.partition("?")[0]
            if path == "/":
                return self.respond(
                    200, (TOOL_DIR / "web/editor.html").read_bytes(), "text/html; charset=utf-8"
                )
            if path in ("/static/editor.js", "/static/editor.css"):
                mime = "text/javascript" if path.endswith(".js") else "text/css"
                return self.respond(
                    200, (TOOL_DIR / "web" / path.rsplit("/", 1)[1]).read_bytes(), mime + "; charset=utf-8"
                )
            if path == "/api/config":
                return self.respond(
                    200,
                    {
                        "token": self.server.token,
                        "max_source_bytes": MAX_SOURCE_BYTES,
                        "max_asset_bytes": MAX_ASSET_BYTES,
                        "max_assets": MAX_ASSETS,
                        "timeout_s": self.server.service.timeout_s,
                        "result_limit": RESULT_LIMIT,
                        "examples": [{"id": key, "label": value[0]} for key, value in EXAMPLES.items()],
                    },
                )
            if path.startswith("/api/examples/"):
                return self.respond(200, self.server.service.example(path.rsplit("/", 1)[1]))
            match = re.fullmatch(
                r"/(results|downloads)/([a-f0-9]{32})/(viewer\.html|program\.yaml|program\.zip)", path
            )
            if match:
                kind, result_id, name = match.groups()
                source = name == "program.yaml"
                bundle = name == "program.zip"
                if kind == "results" and (source or bundle):
                    raise EditorError(404, "Unbekannte Ansicht.")
                raw, filename = self.server.service.result_file(result_id, source, bundle)
                return self.respond(
                    200,
                    raw,
                    (
                        "application/zip"
                        if bundle
                        else "text/yaml; charset=utf-8" if source else "text/html; charset=utf-8"
                    ),
                    (
                        (
                            Path(filename).stem + "_programm.zip"
                            if bundle
                            else filename if source else Path(filename).stem + "_simulation.html"
                        )
                        if kind == "downloads"
                        else None
                    ),
                )
            raise EditorError(404, "Unbekannte Adresse.")
        except EditorError as error:
            self.respond(error.status, {"error": error.diagnostic})

    def do_POST(self):
        try:
            self.check_host()
            origin = self.headers.get("Origin")
            if origin is not None and origin not in {"http://" + host for host in self.server.hosts}:
                raise EditorError(403, "Die Anfrage muss aus dem lokalen Editor stammen.")
            if self.path not in ("/api/simulate", "/api/cancel"):
                raise EditorError(404, "Unbekannte Adresse.")
            token = self.headers.get("X-Motion-Token", "")
            if not token.isascii() or not secrets.compare_digest(token, self.server.token):
                raise EditorError(403, "Editor-Sitzung ungültig; Seite neu laden.")
            if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
                raise EditorError(415, "JSON-Anfrage erwartet.")
            length = self.headers.get("Content-Length", "")
            if (
                self.headers.get("Transfer-Encoding")
                or not length.isdigit()
                or not 0 < int(length) <= MAX_REQUEST_BYTES
            ):
                raise EditorError(413, "Ungültige Anfragegröße; maximal 12 MiB.")
            raw = self.rfile.read(int(length))
            if len(raw) != int(length):
                raise EditorError(400, "Unvollständige Anfrage.")
            try:
                request = json.loads(
                    raw.decode("utf-8"), object_pairs_hook=unique_object, parse_constant=reject_constant
                )
            except (UnicodeError, ValueError) as error:
                raise EditorError(400, "Ungültiges JSON: " + str(error)) from error
            if self.path == "/api/cancel":
                if not isinstance(request, dict) or set(request) != {"job_id"}:
                    raise EditorError(400, "Abbruch benötigt genau job_id.")
                return self.respond(200, self.server.service.cancel(request["job_id"]))
            self.respond(200, self.server.service.simulate(request))
        except EditorError as error:
            self.respond(error.status, {"error": error.diagnostic})
        except (TimeoutError, ConnectionError):
            self.close_connection = True
