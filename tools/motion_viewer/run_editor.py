"""Build and launch the local YAML program editor with native C++ simulation."""

from pathlib import Path
import argparse
import webbrowser
from create_viewer import ROOT
from editor_server import EditorServer, SimulationService
from tool_build import build_exporter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765, help="Local port, 0 chooses a free port")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build-motion-viewer")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--exporter", type=Path, help="Use an already built exporter; skip the build")
    parser.add_argument("--timeout-s", type=int, default=120, help="Native planning/simulation time budget")
    parser.add_argument("--open", action="store_true", help="Open the local editor in the default browser")
    options = parser.parse_args()
    if not 0 <= options.port <= 65535 or not 1 <= options.timeout_s <= 600:
        parser.error("port must be 0..65535; timeout-s must be 1..600")
    exporter = options.exporter or build_exporter(options.build_dir, options.cmake)
    try:
        service = SimulationService(exporter, timeout_s=options.timeout_s)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Editor could not start: {error}\n")
    try:
        server = EditorServer(service, options.port)
    except OSError as error:
        service.close()
        parser.exit(1, f"Editor could not start: {error}. Try --port 8766.\n")
    print(
        f"Motion Editor: {server.url}\nStop with Ctrl+C. Simulation only; no hardware connection.", flush=True
    )
    if options.open:
        webbrowser.open(server.url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()


if __name__ == "__main__":
    main()
