"""Create a standalone offline viewer from the library's admitted program simulation.

Only Python's standard library is required. The native exporter is a separate tool
in this directory; it links the public library without changing its implementation.
"""

from pathlib import Path
import argparse
import csv
import json
import math
import os
import re
import subprocess
import tempfile
from visual_assets import select_visual_model, license_text
from job_budget import JobBudget

TOOL_DIR = Path(__file__).resolve().parent
ROOT = TOOL_DIR.parents[1]
MAX_BYTES = 128 * 1024 * 1024
MAX_SAMPLES = 100000


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, size, label):
    require(
        isinstance(value, list) and len(value) == size and all(finite_number(x) for x in value),
        "Invalid finite vector: " + label,
    )


def validate_pose(value):
    require(isinstance(value, dict), "Invalid pose")
    vector(value.get("position"), 3, "position")
    vector(value.get("orientation"), 4, "orientation")
    require(abs(sum(x * x for x in value["orientation"]) - 1) < 1e-6, "Invalid quaternion norm")


def reject_constant(value):
    raise ValueError("Nonfinite JSON constant: " + value)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "Duplicate JSON key: " + key)
        result[key] = value
    return result


def load_export(filename, budget=None):
    if budget:
        budget.checkpoint()
    filename = Path(filename)
    require(filename.stat().st_size <= MAX_BYTES, "Export exceeds 128 MiB")
    with filename.open(encoding="utf-8") as source:
        result = json.load(source, parse_constant=reject_constant, object_pairs_hook=unique_object)
    if budget:
        budget.checkpoint()
    return result


def trace_lines(text):
    """Yield CSV physical lines without StringIO's full Unicode buffer copy."""
    # Match physical lines lazily, preserving LF, CRLF and legacy CR input.
    # csv.reader may combine these lines for quoted fields before validation.
    for match in re.finditer(r"[^\r\n]*(?:\r\n|[\r\n]|$)", text):
        line = match.group()
        if line:
            yield line


def prepare_data(exported, max_frames=4000, visuals="auto", budget=None):
    if budget:
        budget.checkpoint()
    require(2 <= max_frames <= 20000, "max_frames must be 2..20000")
    require(
        isinstance(exported, dict)
        and exported.get("schema_version") == 1
        and exported.get("format") == "robot-motion-viewer",
        "Unsupported visualization export",
    )
    robot = exported["robot"]
    program = exported["program"]
    require(isinstance(robot["name"], str) and len(robot["name"]) <= 4096, "Invalid robot name")
    validate_pose(robot["base_pose"])
    validate_pose(robot["tool_offset"])
    names = robot["joint_names"]
    require(
        isinstance(names, list)
        and 1 <= len(names) <= 12
        and all(isinstance(name, str) and 0 < len(name) <= 4096 for name in names)
        and len(set(names)) == len(names),
        "Invalid joint names",
    )
    moving = []
    require(isinstance(robot["joints"], list) and len(robot["joints"]) <= 1024, "Invalid model chain")
    for joint in robot["joints"]:
        require(joint["type"] in ("fixed", "revolute", "continuous", "prismatic"), "Unsupported joint type")
        validate_pose(joint["origin"])
        vector(joint["axis"], 3, "joint axis")
        if joint["type"] != "fixed":
            require(abs(math.hypot(*joint["axis"]) - 1) < 1e-8, "Joint axis must be normalized")
            moving.append(joint)
    require([joint["name"] for joint in moving] == names, "Moving joint order mismatch")
    for key in ("lower", "upper", "velocity", "acceleration", "jerk"):
        vector(robot["limits"][key], len(names), key)
    for axis in range(len(names)):
        require(
            robot["limits"]["lower"][axis] < robot["limits"]["upper"][axis], "Invalid joint position window"
        )
        require(
            all(robot["limits"][key][axis] > 0 for key in ("velocity", "acceleration", "jerk")),
            "Invalid joint derivative cap",
        )
    for group in ("linear", "angular"):
        vector(robot["tcp_limits"][group], 3, group + " TCP caps")
        require(all(x > 0 for x in robot["tcp_limits"][group]), "Invalid TCP caps")
    require(
        finite_number(program["cycle_s"])
        and program["cycle_s"] > 0
        and finite_number(program["duration_s"])
        and program["duration_s"] > 0
        and isinstance(program["total_ticks"], int)
        and 0 < program["total_ticks"] < MAX_SAMPLES,
        "Invalid program timing",
    )
    segments = program["segments"]
    require(isinstance(segments, list) and 0 < len(segments) <= 10000, "Invalid segment list")
    previous_end = 0
    for index, segment in enumerate(segments, 1):
        require(
            segment["index"] == index
            and segment["operation"] in ("PTP", "LIN", "CIRC", "WAIT")
            and finite_number(segment["start_s"])
            and finite_number(segment["end_s"])
            and segment["start_s"] == previous_end
            and segment["end_s"] > previous_end,
            "Invalid segment ordering",
        )
        if "orientation_mode" in segment:
            require(segment["operation"] == "LIN" and segment["orientation_mode"] in ("keep", "interpolate"),
                    "Invalid LIN orientation mode")
        require(finite_number(segment["speed"]) and 0.001 <= segment["speed"] <= 1, "Invalid speed factor")
        for key in ("source_file", "source_block"):
            require(isinstance(segment[key], str) and len(segment[key]) <= 8192, "Invalid source label")
        if "source_command" in segment:
            require(segment["source_command"] == "DXF", "Unsupported source command")
            for key in ("dxf_file", "dxf_layer", "dxf_type"):
                require(
                    isinstance(segment.get(key), str) and len(segment[key]) <= 4096,
                    "Invalid DXF source label",
                )
            for key in ("dxf_entity", "dxf_line"):
                require(type(segment.get(key)) is int and segment[key] > 0, "Invalid DXF source location")
        previous_end = segment["end_s"]
    require(previous_end == program["duration_s"], "Segment duration mismatch")
    trace = exported["trace_csv"]
    require(isinstance(trace, str) and len(trace) <= 64 * 1024 * 1024, "Invalid CSV payload size")
    reader = csv.reader(trace_lines(trace))
    columns = next(reader, [])
    require(columns and len(columns) == len(set(columns)), "Missing or duplicate CSV columns")
    lookup = {name: index for index, name in enumerate(columns)}
    required = [
        "tick",
        "time_s",
        "timestamp_s",
        "segment",
        "source_step",
        "tcp_x",
        "tcp_y",
        "tcp_z",
        "qw",
        "qx",
        "qy",
        "qz",
    ]
    required += [prefix + str(axis) for prefix in ("q", "dq", "ddq", "jerk") for axis in range(len(names))]
    required += [prefix + str(axis) for prefix in ("tcp_v", "tcp_a", "tcp_j") for axis in range(6)]
    require(all(key in lookup for key in required), "Missing required CSV columns")
    # First pass validates EVERY row, retaining only scalar state and boundary indices.
    sample_count = 0
    previous_time = None
    previous_segment = None
    mandatory = {0, program["total_ticks"]}
    violations = []
    peak_ratios = {
        "joint_velocity": 0.0,
        "joint_acceleration": 0.0,
        "joint_jerk": 0.0,
        "tcp_linear": [0.0, 0.0, 0.0],
        "tcp_angular": [0.0, 0.0, 0.0],
    }
    for row in reader:
        if budget:
            budget.checkpoint()
        require(sample_count < MAX_SAMPLES and len(row) == len(columns), "CSV row/sample budget mismatch")
        values = [float(value) for value in row]
        require(all(math.isfinite(value) for value in values), "Nonfinite trace value")
        sample_index = sample_count
        get = lambda name: values[lookup[name]]
        require(get("tick") == sample_index, "Missing or duplicate tick")
        require(get("segment").is_integer() and 1 <= get("segment") <= len(segments), "Invalid segment index")
        require(get("source_step").is_integer() and get("source_step") > 0, "Invalid source step")
        require(
            math.isclose(get("time_s"), sample_index * program["cycle_s"], rel_tol=1e-12, abs_tol=1e-10),
            "Tick time mismatch",
        )
        if previous_time is not None:
            require(get("time_s") > previous_time, "Time must increase")
        else:
            epoch_s = get("timestamp_s")
        require(
            math.isclose(
                get("timestamp_s"),
                epoch_s + get("time_s"),
                rel_tol=0,
                abs_tol=max(1e-12, 2 * math.ulp(get("timestamp_s"))),
            ),
            "Timestamp mismatch",
        )
        quaternion = [get("q" + axis) for axis in "wxyz"]
        require(abs(sum(x * x for x in quaternion) - 1) < 1e-6, "Invalid trace quaternion")
        for axis in range(len(names)):
            position = get("q" + str(axis))
            if not robot["limits"]["lower"][axis] - 1e-8 <= position <= robot["limits"]["upper"][axis] + 1e-8:
                if len(violations) < 20:
                    violations.append({"tick": sample_index, "channel": "q" + str(axis)})
            for prefix, cap_key, metric in [
                ("dq", "velocity", "joint_velocity"),
                ("ddq", "acceleration", "joint_acceleration"),
                ("jerk", "jerk", "joint_jerk"),
            ]:
                magnitude = abs(get(prefix + str(axis)))
                cap = robot["limits"][cap_key][axis]
                peak_ratios[metric] = max(peak_ratios[metric], magnitude / cap)
                if magnitude > cap + 1e-8 and len(violations) < 20:
                    violations.append({"tick": sample_index, "channel": prefix + str(axis)})
        for order, prefix in enumerate(("tcp_v", "tcp_a", "tcp_j")):
            for group, begin in [("linear", 0), ("angular", 3)]:
                magnitude = math.hypot(*(get(prefix + str(axis)) for axis in range(begin, begin + 3)))
                cap = robot["tcp_limits"][group][order]
                peak_ratios["tcp_" + group][order] = max(peak_ratios["tcp_" + group][order], magnitude / cap)
                if magnitude > cap + 1e-8 and len(violations) < 20:
                    violations.append({"tick": sample_index, "channel": prefix + "_" + group})
        current_segment = get("segment")
        if previous_segment is not None and current_segment != previous_segment:
            mandatory.update((sample_index - 1, sample_index))
        previous_segment = current_segment
        previous_time = get("time_s")
        sample_count += 1
    require(sample_count == program["total_ticks"] + 1 == program["commands"], "Incomplete trace")
    require(
        math.isclose(previous_time, program["duration_s"], abs_tol=1e-10), "End time mismatch"
    )
    if sample_count <= max_frames:
        chosen = set(range(sample_count))
    else:
        require(len(mandatory) < max_frames, "Too many boundaries; increase --max-frames")
        stride = math.ceil(sample_count / (max_frames - len(mandatory)))
        chosen = mandatory | set(range(0, sample_count, stride))
    # Second pass keeps only selected display rows and reuses the immutable
    # CSV string. Neither pass copies the full payload into StringIO.
    reader = csv.reader(trace_lines(trace))
    next(reader)
    frames = []
    for index, row in enumerate(reader):
        if budget:
            budget.checkpoint()
        if index in chosen:
            frames.append([float(value) for value in row])
    result = {key: exported[key] for key in ("format", "schema_version", "robot", "program")}
    if "preparation" in exported:
        metrics = exported["preparation"]
        require(isinstance(metrics, dict), "Invalid preparation metrics")
        for key in ("planning_s", "simulation_s"):
            require(finite_number(metrics.get(key)) and metrics[key] >= 0, "Invalid preparation time")
        require(type(metrics.get("work_units")) is int and metrics["work_units"] >= 0,
                "Invalid preparation work count")
        require(isinstance(metrics.get("build_configuration"), str)
                and len(metrics["build_configuration"]) <= 64, "Invalid build configuration")
        result["preparation"] = {key: metrics[key] for key in
                                 ("planning_s", "simulation_s", "work_units", "build_configuration")}
    if budget:
        budget.checkpoint()
    visual_model = select_visual_model(robot, visuals)
    if budget:
        budget.checkpoint()
    if visual_model is not None:
        result["visual_model"] = visual_model
        result["legal_notices"] = license_text()
    result.update(
        columns=columns,
        frames=frames,
        original_frame_count=sample_count,
        display_frame_count=len(chosen),
        sample_limit_violations=violations,
        peak_ratios=peak_ratios,
    )
    return result


def render_html(data):
    payload = json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    for source, escaped in [
        ("&", "\\u0026"),
        ("<", "\\u003c"),
        (">", "\\u003e"),
        ("\u2028", "\\u2028"),
        ("\u2029", "\\u2029"),
    ]:
        payload = payload.replace(source, escaped)
    template = (TOOL_DIR / "web/viewer.html").read_text(encoding="utf-8")
    vendor = (TOOL_DIR / "vendor/three.min.js").read_text(encoding="utf-8") if data.get("visual_model") else ""
    mesh_code = (TOOL_DIR / "web/mesh_renderer.js").read_text(encoding="utf-8") if data.get("visual_model") else ""
    return (
        template.replace("/*__THREE__*/", vendor)
        .replace("/*__MESH_RENDERER__*/", mesh_code)
        .replace("/*__VIEWER_STYLE__*/", (TOOL_DIR / "web/viewer.css").read_text(encoding="utf-8"))
        .replace("/*__KINEMATICS__*/", (TOOL_DIR / "web/kinematics.js").read_text(encoding="utf-8"))
        .replace("/*__VIEWER_APP__*/", (TOOL_DIR / "web/viewer.js").read_text(encoding="utf-8"))
        .replace("__VIEWER_DATA__", payload)
    )


def atomic_write(destination, content):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=destination.name + ".tmp-",
            delete=False,
        ) as file:
            temporary = Path(file.name)
            file.write(content)
        os.replace(temporary, destination)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def export_and_render(
    program, exporter, destination, cycle_ms=10, max_frames=4000, timeout_s=180, visuals="auto"
):
    require(Path(destination).suffix.lower() == ".html", "Output needs .html extension")
    require(Path(destination).resolve() != Path(program).resolve(), "Input and output must differ")
    require(Path(exporter).is_file(), "Exporter missing; first build tools/motion_viewer")
    with tempfile.TemporaryDirectory(prefix="robot-motion-viewer-") as directory:
        budget = JobBudget(timeout_s)
        exported = Path(directory) / "scene.json"
        environment = os.environ.copy()
        environment["MOTION_PREPARATION_TIMEOUT_MS"] = str(max(1, int(budget.remaining() * 1000)))
        subprocess.run(
            [str(Path(exporter).resolve()), str(Path(program).resolve()), str(exported), str(cycle_ms)],
            check=True,
            timeout=budget.remaining(), env=environment,
        )
        data = prepare_data(load_export(exported, budget=budget), max_frames, visuals, budget=budget)
        html = render_html(data)
        budget.checkpoint()
        atomic_write(destination, html)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("program", nargs="?", type=Path)
    parser.add_argument("--from-export", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--exporter",
        type=Path,
        default=ROOT
        / "build-motion-viewer"
        / ("motion_viewer_exporter.exe" if os.name == "nt" else "motion_viewer_exporter"),
    )
    parser.add_argument("--cycle-ms", type=int, choices=range(1, 21), default=10)
    parser.add_argument("--max-frames", type=int, default=4000)
    parser.add_argument("--visuals", choices=["auto", "none", "ur5e"], default="auto")
    options = parser.parse_args()
    if bool(options.program) == bool(options.from_export):
        parser.error("Supply PROGRAM.yaml or --from-export")
    if options.output.suffix.lower() != ".html":
        parser.error("Output needs .html extension")
    try:
        if options.from_export:
            require(options.from_export.resolve() != options.output.resolve(), "Input and output must differ")
            budget = JobBudget(180)
            data = prepare_data(load_export(options.from_export, budget=budget), options.max_frames, options.visuals, budget=budget)
            html = render_html(data)
            budget.checkpoint()
            atomic_write(options.output, html)
        else:
            data = export_and_render(
                options.program,
                options.exporter,
                options.output,
                options.cycle_ms,
                options.max_frames,
                visuals=options.visuals,
            )
    except (ValueError, KeyError, TypeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Viewer creation failed: {error}\n")
    print(
        f'{options.output.resolve()}\n{data["display_frame_count"]} displayed / {data["original_frame_count"]} fully checked samples; offline simulation viewer'
    )


if __name__ == "__main__":
    main()
