"""Measure preparation separately from simulation; never dispatches hardware commands."""
from pathlib import Path
import argparse
import hashlib
import json
import platform
import statistics
import subprocess
import tempfile
import time

from create_viewer import ROOT, atomic_write, finite_number, load_export, require
from tool_build import build_exporter


def benchmark_program(exporter, program, cycle_ms, repeats):
    runs = []
    identity = None
    with tempfile.TemporaryDirectory(prefix="motion-benchmark-") as folder:
        output = Path(folder) / "scene.json"
        for _ in range(repeats):
            started = time.perf_counter()
            subprocess.run([str(exporter), str(program), str(output), str(cycle_ms)],
                           capture_output=True, text=True, check=True, timeout=180)
            wall_s = time.perf_counter() - started
            scene = load_export(output)
            require(scene.get("format") == "robot-motion-viewer", "Invalid exporter format")
            metrics = scene.get("preparation", {})
            for key in ("planning_s", "simulation_s"):
                require(finite_number(metrics.get(key)) and metrics[key] >= 0,
                        "Exporter needs preparation metrics; rebuild the current version")
            duration_s = scene["program"]["duration_s"]
            require(finite_number(duration_s) and duration_s > 0, "Invalid motion duration")
            require(type(metrics.get("work_units")) is int and metrics["work_units"] >= 0,
                    "Invalid preparation work count")
            require(isinstance(metrics.get("build_configuration"), str), "Missing build configuration")
            current = (duration_s, scene["program"]["commands"], metrics["work_units"],
                       metrics["build_configuration"],
                       hashlib.sha256(scene["trace_csv"].encode("utf-8")).hexdigest())
            require(identity is None or identity == current, "Nondeterministic benchmark trajectory")
            identity = current
            runs.append({"planning_s": metrics["planning_s"],
                         "simulation_s": metrics["simulation_s"], "native_process_s": wall_s})
            output.unlink()
    duration_s, commands, work, configuration, trace_hash = identity
    medians = {key: statistics.median(run[key] for run in runs) for key in runs[0]}
    return {"program": str(program), "source_sha256": hashlib.sha256(program.read_bytes()).hexdigest(),
            "cycle_ms": cycle_ms, "duration_s": duration_s, "commands": commands, "work_units": work,
            "build_configuration": configuration, "trace_sha256": trace_hash, "runs": runs,
            "median": medians, "maximum_planning_s": max(run["planning_s"] for run in runs),
            "planning_to_motion_ratio": medians["planning_s"] / duration_s}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exporter", type=Path, help="Use an existing exporter; otherwise build Release")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build-motion-viewer")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--program", type=Path, action="append", help="Repeat for several programs")
    parser.add_argument("--cycle-ms", type=int, nargs="+", choices=range(1, 21), default=[10])
    parser.add_argument("--repeats", type=int, choices=range(1, 21), default=3)
    parser.add_argument("--output", type=Path, default=ROOT / "build-motion-viewer/benchmark.json")
    options = parser.parse_args()
    exporter = (options.exporter or build_exporter(options.build_dir, options.cmake)).resolve()
    programs = options.program or [ROOT / "programs/examples/ur5e_pick_place.yaml",
                                  ROOT / "programs/examples/ur5e_lin_keep_orientation.yaml"]
    if options.output.resolve() in {exporter, *(program.resolve() for program in programs)}:
        parser.error("Benchmark report must not overwrite an input program or executable")
    report = {"format": "robot-motion-preparation-benchmark", "schema_version": 1,
              "platform": platform.platform(), "machine": platform.machine(), "cases": []}
    try:
        for program in programs:
            for cycle in options.cycle_ms:
                case = benchmark_program(exporter, program.resolve(), cycle, options.repeats)
                report["cases"].append(case)
                print(f'{program.name} [{case["build_configuration"]}, {cycle} ms]: '
                      f'plan {case["median"]["planning_s"]:.3f} s, '
                      f'simulation/CSV {case["median"]["simulation_s"]:.3f} s, '
                      f'motion {case["duration_s"]:.3f} s', flush=True)
    except subprocess.CalledProcessError as error:
        parser.exit(1, "Native preparation failed: " + error.stderr[-4000:] + "\n")
    atomic_write(options.output, json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f'Report: {options.output.resolve()} (host measurements, no real-time guarantee)')


if __name__ == "__main__":
    main()
