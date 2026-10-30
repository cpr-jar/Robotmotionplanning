"""Select the offline visual pack only when its kinematic reference matches.

This is a viewer feature. It does not alter the robot model used by the planner.
"""

from pathlib import Path
import hashlib
import json

TOOL_DIR = Path(__file__).resolve().parent
ASSETS = TOOL_DIR / "assets/ur5e"


def compatible(robot, reference):
    if robot.get("base_link") != reference["base_link"] or robot.get("tip_link") != reference["tip_link"]:
        return False
    actual = robot.get("joints", [])
    if len(actual) != len(reference["joints"]):
        return False
    for joint, expected in zip(actual, reference["joints"]):
        if any(joint.get(key) != expected[key] for key in ("name", "type", "parent_link", "child_link")):
            return False
        if any(abs(a - b) > 1e-9 for a, b in zip(joint["axis"], expected["axis"])):
            return False
        if any(
            abs(a - b) > 1e-9 for a, b in zip(joint["origin"]["position"], expected["origin"]["position"])
        ):
            return False
        q = joint["origin"]["orientation"]
        p = expected["origin"]["orientation"]
        if abs(abs(sum(a * b for a, b in zip(q, p))) - 1) > 1e-10:
            return False
    return True


def select_visual_model(robot, mode="auto"):
    if mode not in ("auto", "none", "ur5e"):
        raise ValueError("Unknown visual model selection")
    if mode == "none":
        return None
    index = json.loads((ASSETS / "manifest.json").read_text())
    if not compatible(robot, index["reference"]):
        if mode == "ur5e":
            raise ValueError("UR5e visual model does not match the imported kinematic chain")
        return None
    path = ASSETS / "visuals.json"
    if path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError("Visual pack exceeds 32 MiB")
    raw = path.read_bytes().replace(b"\r\n", b"\n")
    if hashlib.sha256(raw).hexdigest() != index["sha256"]:
        raise ValueError("Visual asset checksum mismatch")
    model = json.loads(raw)
    if model["triangle_count"] != index["triangle_count"] or len(model["links"]) != 7:
        raise ValueError("Invalid visual model manifest")
    return model


def license_text():
    ur = (ASSETS / "source/LICENSE").read_text(encoding="utf-8")
    three = (TOOL_DIR / "vendor/THREE-LICENSE.txt").read_text(encoding="utf-8")
    index = json.loads((ASSETS / "sources.json").read_text())
    authors = (
        (ASSETS / "source/urdf/ur_macro.xacro").read_text(encoding="utf-8").split("<!--", 1)[1].split("-->", 1)[0].strip()
    )
    return (
        "UR5e visual geometry and description\n" + index["repository"] + "\nCommit " + index["commit"] + "\n"
        "Original files and author notices are retained under assets/ur5e/source/.\n\n"
        + authors
        + "\n\n"
        + ur
        + "\n\nThree.js r160 (0.160.0)\nhttps://github.com/mrdoob/three.js\n"
        + three
    )
