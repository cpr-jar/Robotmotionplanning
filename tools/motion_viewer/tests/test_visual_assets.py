"""Integrity, frame compatibility and conversion checks for offline UR visuals."""

from pathlib import Path
import base64
import copy
import hashlib
import json
import math
import struct
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import build_visual_assets as builder
import visual_assets as assets


def unpack(encoded, kind):
    raw = base64.b64decode(encoded, validate=True)
    return struct.unpack("<" + kind * (len(raw) // 4), raw)


class VisualAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.index = json.loads((assets.ASSETS / "manifest.json").read_text())
        cls.robot = copy.deepcopy(cls.index["reference"])
        cls.model = assets.select_visual_model(cls.robot, "ur5e")

    def test_all_pinned_sources_and_vendor_files_match_checksums(self):
        for manifest, folder, key in [
            (assets.ASSETS / "sources.json", assets.ASSETS / "source", "path"),
            (assets.TOOL_DIR / "vendor/manifest.json", assets.TOOL_DIR / "vendor", "file"),
        ]:
            for record in json.loads(manifest.read_text())["files"]:
                raw = (folder / record[key]).read_bytes()
                self.assertEqual(len(raw), record["bytes"])
                self.assertEqual(hashlib.sha256(raw).hexdigest(), record["sha256"])

    def test_visual_pack_accepts_lf_and_crlf_but_rejects_content_changes(self):
        raw = (assets.ASSETS / "visuals.json").read_bytes().replace(b"\r\n", b"\n")
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "manifest.json").write_bytes((assets.ASSETS / "manifest.json").read_bytes())
            with mock.patch.object(assets, "ASSETS", folder):
                for newline in (b"\n", b"\r\n"):
                    with self.subTest(newline=newline):
                        (folder / "visuals.json").write_bytes(raw.replace(b"\n", newline))
                        self.assertEqual(assets.select_visual_model(self.robot, "ur5e"), self.model)
                for mode in ("auto", "ur5e"):
                    with self.subTest(mode=mode):
                        (folder / "visuals.json").write_bytes(raw.replace(b'"Z_UP"', b'"Y_UP"'))
                        with self.assertRaisesRegex(ValueError, "Visual asset checksum mismatch"):
                            assets.select_visual_model(self.robot, mode)

    def test_complete_meshes_and_physical_scale(self):
        self.assertEqual(self.model["up_axis"], "Z_UP")
        self.assertEqual(len(self.model["links"]), 7)
        self.assertEqual(self.model["triangle_count"], 121789)
        triangle_counts = [4838, 23290, 40124, 15058, 17288, 20280, 911]
        for link, count in zip(self.model["links"], triangle_counts):
            triangles = 0
            all_positions = []
            for part in link["parts"]:
                positions, normals, indices = (
                    unpack(part[key], kind)
                    for key, kind in [("positions", "f"), ("normals", "f"), ("indices", "I")]
                )
                self.assertEqual(len(positions), part["vertex_count"] * 3)
                self.assertEqual(len(indices), part["index_count"])
                self.assertLess(max(indices), part["vertex_count"])
                self.assertTrue(all(math.isfinite(x) for x in positions + normals))
                self.assertTrue(all(0 <= x <= 1 for x in part["color"]))
                if part["primitive"] == "triangles":
                    triangles += len(indices) // 3
                    self.assertEqual(len(normals), len(positions))
                    self.assertEqual(len(indices) % 3, 0)
                    for index in range(0, len(normals), 3):
                        self.assertAlmostEqual(math.hypot(*normals[index : index + 3]), 1, delta=1e-6)
                else:
                    self.assertEqual(part["primitive"], "lines")
                    self.assertEqual(len(indices) % 2, 0)
                all_positions.extend(positions)
            self.assertEqual(triangles, count)
            # Every component fits the physical UR5e envelope, not millimetres treated as metres.
            span = max(max(all_positions[a::3]) - min(all_positions[a::3]) for a in range(3))
            self.assertGreater(span, 0.05)
            self.assertLess(span, 0.65)

    def test_chain_identity_controls_selection_and_base_tool_are_independent(self):
        self.assertTrue(assets.compatible(self.robot, self.index["reference"]))
        for key, value in [
            ("axis", [0, 1, 0]),
            ("name", "other_joint"),
            ("type", "continuous"),
            ("child_link", "other_link"),
        ]:
            robot = copy.deepcopy(self.robot)
            robot["joints"][1][key] = value
            self.assertFalse(assets.compatible(robot, self.index["reference"]))
            self.assertIsNone(assets.select_visual_model(robot))
            with self.assertRaisesRegex(ValueError, "does not match"):
                assets.select_visual_model(robot, "ur5e")
        robot = copy.deepcopy(self.robot)
        robot["joints"][2]["origin"]["position"][0] += 0.001
        self.assertIsNone(assets.select_visual_model(robot))
        robot = copy.deepcopy(self.robot)
        robot["name"] = "Custom workspace"
        robot["base_pose"] = {"position": [1, 2, 3], "orientation": [0.5] * 4}
        robot["tool_offset"] = {"position": [0, 0, 0.1], "orientation": [1, 0, 0, 0]}
        robot["joints"][2]["origin"]["orientation"] = [
            -x for x in robot["joints"][2]["origin"]["orientation"]
        ]
        self.assertTrue(assets.compatible(robot, self.index["reference"]))
        del robot["joints"][2]["child_link"]  # Older exports lack frame names: skeleton fallback.
        self.assertIsNone(assets.select_visual_model(robot))
        self.assertIsNone(assets.select_visual_model({}, "none"))

    def test_visual_urdf_retains_kinematics_and_resolves_every_mesh(self):
        robot = ET.parse(assets.ASSETS / "ur5e_visual.urdf").getroot()
        original = ET.parse(assets.TOOL_DIR.parents[1] / "models/ur5e/robot.urdf").getroot()
        canonical = lambda node: ET.canonicalize(ET.tostring(node, encoding="unicode"), strip_text=True)
        self.assertEqual(
            [canonical(j) for j in robot.findall("joint")], [canonical(j) for j in original.findall("joint")]
        )
        visuals = robot.findall("link/visual")
        self.assertEqual(len(visuals), 7)
        for visual in visuals:
            self.assertTrue((assets.ASSETS / visual.find("geometry/mesh").get("filename")).is_file())

    def test_converter_preserves_units_scene_offsets_and_inverse_transpose_normals(self):
        # Independent hand-calculated triangle: nonuniform scale, translation, cm units.
        xml = """<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
<asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>
<library_effects><effect id="e"><profile_COMMON><technique><phong><diffuse><color>0.2 0.4 0.6 1</color></diffuse></phong></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="m"><instance_effect url="#e"/></material></library_materials>
<library_geometries><geometry id="g"><mesh>
<source id="p"><float_array>0 0 0 1 0 0 0 1 1</float_array><technique_common><accessor count="3" stride="3"/></technique_common></source>
<source id="n"><float_array>0 -0.7071067811865476 0.7071067811865476</float_array><technique_common><accessor count="1" stride="3"/></technique_common></source>
<vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
<polylist count="1" material="s"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#n" offset="1"/><vcount>3</vcount><p>0 0 1 0 2 0</p></polylist>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node><matrix>1 0 0 10 0 1 0 20 0 0 1 30 0 0 0 1</matrix><node><scale>2 3 4</scale><instance_geometry url="#g"><bind_material><technique_common><instance_material symbol="s" target="#m"/></technique_common></bind_material></instance_geometry></node></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "fixture.dae"
            source.write_text(xml)
            parts = builder.convert_collada(source)
            self.assertEqual(len(parts), 1)
            part = parts[0]
            self.assertEqual(part["color"], [0.2, 0.4, 0.6, 1])
            expected = [0.10, 0.20, 0.30, 0.12, 0.20, 0.30, 0.10, 0.23, 0.34]
            for actual, expected in zip(unpack(part["positions"], "f"), expected):
                self.assertAlmostEqual(actual, expected, places=7)
            self.assertEqual(unpack(part["indices"], "I"), (0, 1, 2))
            for i, actual in enumerate(unpack(part["normals"], "f")):
                self.assertAlmostEqual(actual, [0, -0.8, 0.6][i % 3], places=6)
            source.write_text(xml.replace("Z_UP", "Y_UP"))
            with self.assertRaisesRegex(ValueError, "Z_UP"):
                builder.convert_collada(source)

    def test_html_licenses_include_authors_and_complete_terms(self):
        text = assets.license_text()
        self.assertIn("Ludovic Delval", text)
        self.assertIn((assets.ASSETS / "source/LICENSE").read_text(encoding="utf-8"), text)
        self.assertIn((assets.TOOL_DIR / "vendor/THREE-LICENSE.txt").read_text(encoding="utf-8"), text)


if __name__ == "__main__":
    unittest.main()
