"""Convert pinned UR5e COLLADA visuals to an offline renderer asset pack.

Regeneration uses optional PyYAML. Normal viewer creation needs only the checked-in
JSON pack and Python's standard library. No Xacro, mesh scripts or URDF code runs.
COLLADA scene transforms, meter units, Z-up coordinates, normals and per-material
triangle groups are retained. Unsupported features are rejected explicitly.
"""

from pathlib import Path
import base64
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET

TOOL_DIR = Path(__file__).resolve().parent
ROOT = TOOL_DIR.parents[1]
ASSETS = TOOL_DIR / "assets/ur5e"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def identity():
    return [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def multiply(a, b):
    return [sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4)) for r in range(4) for c in range(4)]


def transform_point(matrix, point):
    return [sum(matrix[r * 4 + c] * point[c] for c in range(3)) + matrix[r * 4 + 3] for r in range(3)]


def transform_normal(matrix, normal):
    a, b, c, d, e, f, g, h, i = (matrix[r * 4 + c] for r in range(3) for c in range(3))
    cofactors = [
        e * i - f * h,
        f * g - d * i,
        d * h - e * g,
        c * h - b * i,
        a * i - c * g,
        b * g - a * h,
        b * f - c * e,
        c * d - a * f,
        a * e - b * d,
    ]
    determinant = a * cofactors[0] + b * cofactors[1] + c * cofactors[2]
    require(abs(determinant) > 1e-18, "Singular COLLADA transform")
    result = [sum(cofactors[r * 3 + c] * normal[c] for c in range(3)) / determinant for r in range(3)]
    length = math.hypot(*result)
    require(length > 0 and math.isfinite(length), "Invalid transformed normal")
    return [value / length for value in result]


def rpy_quaternion(rpy):
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def parse_xml(path):
    data = path.read_bytes()
    require(
        len(data) <= 16 * 1024 * 1024
        and b"<!DOCTYPE" not in data.upper()
        and b"<!ENTITY" not in data.upper(),
        "Unsupported or oversized XML: " + str(path),
    )
    return ET.fromstring(data)


def floats(text):
    result = [float(value) for value in text.split()]
    require(all(math.isfinite(value) for value in result), "Nonfinite mesh value")
    return result


def bounds(positions):
    return [[min(positions[axis::3]) for axis in range(3)], [max(positions[axis::3]) for axis in range(3)]]


def buffer(values, kind):
    return base64.b64encode(struct.pack("<" + kind * len(values), *values)).decode("ascii")


def convert_collada(path):
    root = parse_xml(path)
    require(root.findtext("c:asset/c:up_axis", namespaces=NS) == "Z_UP", "Expected ROS Z_UP mesh")
    meter = float(root.find("c:asset/c:unit", NS).get("meter", "1"))
    require(meter > 0 and math.isfinite(meter), "Invalid COLLADA meter scale")
    require(
        not root.findall(".//c:controller", NS) and not root.findall(".//c:animation", NS),
        "Animated meshes unsupported",
    )
    require(
        not root.findall(".//c:image", NS) and not root.findall(".//c:texture", NS),
        "Texture-bearing meshes unsupported",
    )
    effects = {}
    for effect in root.findall("c:library_effects/c:effect", NS):
        color = effect.find(".//c:diffuse/c:color", NS)
        require(color is not None, "Missing diffuse mesh material")
        rgba = floats(color.text)
        require(len(rgba) == 4 and all(0 <= x <= 1 for x in rgba), "Invalid material color")
        effects[effect.get("id")] = rgba
    materials = {
        material.get("id"): effects[material.find("c:instance_effect", NS).get("url").removeprefix("#")]
        for material in root.findall("c:library_materials/c:material", NS)
    }
    geometries = {}
    for geometry in root.findall("c:library_geometries/c:geometry", NS):
        mesh = geometry.find("c:mesh", NS)
        require(mesh is not None, "Missing mesh")
        sources = {}
        for source in mesh.findall("c:source", NS):
            accessor = source.find("c:technique_common/c:accessor", NS)
            values = floats(source.find("c:float_array", NS).text)
            stride = int(accessor.get("stride", "1"))
            count = int(accessor.get("count"))
            offset = int(accessor.get("offset", "0"))
            require(
                stride >= 3 and offset >= 0 and offset + count * stride <= len(values),
                "Invalid mesh accessor",
            )
            sources[source.get("id")] = (values, stride, count, offset)
        vertices = {
            vertex.get("id"): vertex.find('c:input[@semantic="POSITION"]', NS).get("source").removeprefix("#")
            for vertex in mesh.findall("c:vertices", NS)
        }
        primitives = []
        for primitive in mesh:
            tag = primitive.tag.split("}")[-1]
            if tag in ("source", "vertices"):
                continue
            require(tag in ("polylist", "triangles", "lines"), "Unsupported COLLADA primitive: " + tag)
            inputs = {
                entry.get("semantic"): (int(entry.get("offset", "0")), entry.get("source").removeprefix("#"))
                for entry in primitive.findall("c:input", NS)
            }
            require(
                "VERTEX" in inputs and (tag == "lines" or "NORMAL" in inputs),
                "Positions and triangle normals required",
            )
            require(set(inputs) <= {"VERTEX", "NORMAL", "TEXCOORD", "COLOR"}, "Unsupported mesh input")
            width = max(value[0] for value in inputs.values()) + 1
            indices = [int(value) for value in primitive.find("c:p", NS).text.split()]
            counts = (
                [2 if tag == "lines" else 3] * int(primitive.get("count"))
                if tag in ("triangles", "lines")
                else [int(value) for value in primitive.find("c:vcount", NS).text.split()]
            )
            require(
                len(counts) == int(primitive.get("count"))
                and all((n == 2 if tag == "lines" else 3 <= n <= 64) for n in counts)
                and sum(counts) * width == len(indices),
                "Invalid polygon indices",
            )
            p_offset, p_source = inputs["VERTEX"]
            p_source = vertices[p_source]
            n_offset, n_source = inputs.get("NORMAL", (0, None))
            triangles = []
            cursor = 0
            for size in counts:
                polygon = [
                    (
                        indices[cursor + j * width + p_offset],
                        indices[cursor + j * width + n_offset] if n_source else -1,
                    )
                    for j in range(size)
                ]
                if tag == "lines":
                    triangles.extend(polygon)
                else:
                    for j in range(1, size - 1):
                        triangles.extend((polygon[0], polygon[j], polygon[j + 1]))
                cursor += size * width
            primitives.append(
                (
                    primitive.get("material"),
                    sources[p_source],
                    sources[n_source] if n_source else None,
                    triangles,
                    "lines" if tag == "lines" else "triangles",
                )
            )
        geometries[geometry.get("id")] = primitives
    selected = root.find("c:scene/c:instance_visual_scene", NS).get("url").removeprefix("#")
    scene = next(
        scene
        for scene in root.findall("c:library_visual_scenes/c:visual_scene", NS)
        if scene.get("id") == selected
    )
    parts = []

    def walk(node, parent, depth=0):
        require(depth <= 32, "COLLADA scene too deep")
        local = identity()
        for child in node:
            tag = child.tag.split("}")[-1]
            if tag == "matrix":
                matrix = floats(child.text)
                require(len(matrix) == 16, "Invalid node matrix")
                local = multiply(local, matrix)
            elif tag in ("translate", "rotate", "scale"):
                values = floats(child.text)
                matrix = identity()
                if tag == "translate":
                    require(len(values) == 3, "Invalid translation")
                    matrix[3], matrix[7], matrix[11] = values
                elif tag == "scale":
                    require(len(values) == 3, "Invalid scale")
                    matrix[0], matrix[5], matrix[10] = values
                else:
                    require(len(values) == 4, "Invalid rotation")
                    x, y, z = values[:3]
                    length = math.hypot(x, y, z)
                    require(length > 0, "Invalid rotation axis")
                    x, y, z = x / length, y / length, z / length
                    c = math.cos(math.radians(values[3]))
                    s = math.sin(math.radians(values[3]))
                    d = 1 - c
                    matrix = [
                        c + x * x * d,
                        x * y * d - z * s,
                        x * z * d + y * s,
                        0,
                        y * x * d + z * s,
                        c + y * y * d,
                        y * z * d - x * s,
                        0,
                        z * x * d - y * s,
                        z * y * d + x * s,
                        c + z * z * d,
                        0,
                        0,
                        0,
                        0,
                        1,
                    ]
                local = multiply(local, matrix)
        matrix = multiply(parent, local)
        for instance in node.findall("c:instance_geometry", NS):
            binding = {
                item.get("symbol"): item.get("target").removeprefix("#")
                for item in instance.findall(".//c:instance_material", NS)
            }
            for material, positions, normals, triangles, kind in geometries[
                instance.get("url").removeprefix("#")
            ]:
                points = []
                vectors = []
                output_indices = []
                unique = {}

                def source_value(source, index):
                    values, stride, count, offset = source
                    require(0 <= index < count, "Mesh index outside source")
                    return values[offset + index * stride : offset + index * stride + 3]

                for key in triangles:
                    if key not in unique:
                        unique[key] = len(unique)
                        point = transform_point(matrix, source_value(positions, key[0]))
                        points.extend(value * meter for value in point)
                        if normals is not None:
                            vectors.extend(transform_normal(matrix, source_value(normals, key[1])))
                    output_indices.append(unique[key])
                require(
                    points and len(output_indices) % (2 if kind == "lines" else 3) == 0,
                    "Empty or invalid mesh",
                )
                parts.append(
                    {
                        "primitive": kind,
                        "color": (
                            materials[binding.get(material, material)]
                            if material is not None
                            else [0.15, 0.15, 0.15, 1]
                        ),
                        "material": material,
                        "positions": buffer(points, "f"),
                        "normals": buffer(vectors, "f"),
                        "indices": buffer(output_indices, "I"),
                        "vertex_count": len(points) // 3,
                        "index_count": len(output_indices),
                        "bounds": bounds(points),
                    }
                )
        require(not node.findall("c:instance_node", NS), "Referenced scene nodes unsupported")
        for child in node.findall("c:node", NS):
            walk(child, matrix, depth + 1)

    for node in scene.findall("c:node", NS):
        walk(node, identity())
    require(parts, "No visual geometry found")
    return parts


def reference_chain():
    robot = ET.parse(ROOT / "models/ur5e/robot.urdf").getroot()
    incoming = {j.find("child").get("link"): j for j in robot.findall("joint")}
    chain = []
    link = "tool0"
    while link != "base_link":
        joint = incoming[link]
        origin = joint.find("origin")
        axis = joint.find("axis")
        xyz = floats(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
        rpy = floats(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
        direction = floats(axis.get("xyz")) if axis is not None else [1, 0, 0]
        length = math.hypot(*direction)
        direction = [v / length for v in direction]
        chain.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent_link": joint.find("parent").get("link"),
                "child_link": link,
                "origin": {"position": xyz, "orientation": rpy_quaternion(rpy)},
                "axis": direction,
            }
        )
        link = joint.find("parent").get("link")
    return robot, list(reversed(chain))


def main():
    import yaml

    class VisualLoader(yaml.SafeLoader):
        pass

    VisualLoader.add_constructor(
        "!degrees", lambda loader, node: math.radians(float(loader.construct_scalar(node)))
    )
    manifest = json.loads((ASSETS / "sources.json").read_text())
    for record in manifest["files"]:
        require(
            hashlib.sha256((ASSETS / "source" / record["path"]).read_bytes()).hexdigest() == record["sha256"],
            "Source hash mismatch: " + record["path"],
        )
    config = yaml.load(
        (ASSETS / "source/config/ur5e/visual_parameters.yaml").read_text(), Loader=VisualLoader
    )["mesh_files"]
    robot, chain = reference_chain()
    links = []
    for name, link in [
        ("base", "base_link_inertia"),
        ("shoulder", "shoulder_link"),
        ("upper_arm", "upper_arm_link"),
        ("forearm", "forearm_link"),
        ("wrist_1", "wrist_1_link"),
        ("wrist_2", "wrist_2_link"),
        ("wrist_3", "wrist_3_link"),
    ]:
        entry = config[name]
        mesh = ASSETS / "source" / entry["visual"]["mesh"]["path"]
        offset = entry["mesh_offset"]
        xyz = [offset[key] for key in ("x", "y", "z")]
        rpy = [offset[key] for key in ("roll", "pitch", "yaw")]
        parts = convert_collada(mesh)
        links.append(
            {"link": link, "origin": {"position": xyz, "orientation": rpy_quaternion(rpy)}, "parts": parts}
        )
        visual = ET.SubElement(robot.find(f"link[@name='{link}']"), "visual")
        ET.SubElement(visual, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename="source/" + entry["visual"]["mesh"]["path"])
        print(
            link,
            "parts",
            len(parts),
            "triangles",
            sum(part["index_count"] // 3 for part in parts if part["primitive"] == "triangles"),
        )
    model = {
        "schema_version": 1,
        "model": "ur5e",
        "encoding": "base64-le-f32-u32",
        "up_axis": "Z_UP",
        "reference": {"base_link": "base_link", "tip_link": "tool0", "joints": chain},
        "links": links,
        "provenance": {"repository": manifest["repository"], "commit": manifest["commit"]},
        "triangle_count": sum(
            p["index_count"] // 3 for link in links for p in link["parts"] if p["primitive"] == "triangles"
        ),
    }
    (ASSETS / "visuals.json").write_text(json.dumps(model, separators=(",", ":")) + "\n")
    (ASSETS / "manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "model": "ur5e",
                "reference": model["reference"],
                "sha256": hashlib.sha256((ASSETS / "visuals.json").read_bytes()).hexdigest(),
                "triangle_count": model["triangle_count"],
                "link_count": len(links),
            },
            indent=2,
        )
        + "\n"
    )
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(ASSETS / "ur5e_visual.urdf", encoding="utf-8", xml_declaration=True)
    print(
        "Packed",
        len(links),
        "complete visual links;",
        model["triangle_count"],
        "triangles;",
        (ASSETS / "visuals.json").stat().st_size,
        "bytes",
    )


if __name__ == "__main__":
    main()
