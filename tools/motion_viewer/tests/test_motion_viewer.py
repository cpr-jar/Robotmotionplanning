from pathlib import Path
import copy
import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser

import create_viewer as viewer

EXPORTER = Path(os.environ.get('MOTION_VIEWER_EXPORTER', viewer.ROOT / 'build-motion-viewer/motion_viewer_exporter'))


class ScriptCounter(HTMLParser):
    def __init__(self):
        super().__init__()
        self.count = 0

    def handle_starttag(self, tag, attrs):
        if tag == 'script': self.count += 1


class ViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not EXPORTER.is_file(): raise RuntimeError('Build the standalone motion viewer exporter first')
        cls.directory = tempfile.TemporaryDirectory(prefix='motion-viewer-tests-')
        cls.folder = Path(cls.directory.name)
        cls.demo_file = cls.folder / 'demo.json'
        subprocess.run([str(EXPORTER), str(viewer.TOOL_DIR / 'demo/ur5e_visual_demo.yaml'), str(cls.demo_file), '10'],
                       check=True, capture_output=True, timeout=120)
        cls.exported = viewer.load_export(cls.demo_file)
        cls.full = viewer.prepare_data(cls.exported, 20000)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_complete_simulation_and_limits(self):
        scene = self.full
        self.assertEqual(scene['original_frame_count'], scene['program']['total_ticks'] + 1)
        self.assertEqual({s['operation'] for s in scene['program']['segments']}, {'PTP', 'LIN', 'CIRC', 'WAIT'})
        self.assertFalse(scene['sample_limit_violations'])
        index = {name: i for i, name in enumerate(scene['columns'])}
        for axis in range(len(scene['robot']['joint_names'])):
            self.assertEqual(scene['frames'][-1][index['dq' + str(axis)]], 0)
            self.assertEqual(scene['frames'][-1][index['ddq' + str(axis)]], 0)
            self.assertAlmostEqual(scene['frames'][0][index['q' + str(axis)]],
                                   scene['frames'][-1][index['q' + str(axis)]], places=7)

    def test_decimation_preserves_boundaries_and_full_statistics(self):
        reduced = viewer.prepare_data(self.exported, 40)
        self.assertLessEqual(len(reduced['frames']), 40)
        columns = {name: i for i, name in enumerate(reduced['columns'])}
        ticks = {row[columns['tick']] for row in reduced['frames']}
        self.assertIn(0, ticks)
        self.assertIn(reduced['program']['total_ticks'], ticks)
        for previous, current in zip(self.full['frames'], self.full['frames'][1:]):
            if previous[columns['segment']] != current[columns['segment']]:
                self.assertIn(previous[columns['tick']], ticks)
                self.assertIn(current[columns['tick']], ticks)
        self.assertEqual(reduced['peak_ratios'], self.full['peak_ratios'])

    def test_malformed_input_preserves_existing_html(self):
        bad = copy.deepcopy(self.exported)
        lines = bad['trace_csv'].splitlines()
        lines[2] = lines[1]
        bad['trace_csv'] = '\n'.join(lines)
        source = self.folder / 'bad-export.json'
        source.write_text(json.dumps(bad))
        output = self.folder / 'prior.html'
        output.write_text('prior valid output')
        result = subprocess.run([os.sys.executable, str(viewer.TOOL_DIR / 'create_viewer.py'), '--from-export',
                                 str(source), '--output', str(output)], capture_output=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output.read_text(), 'prior valid output')
        self.assertFalse(list(self.folder.glob('prior.html.tmp-*')))

    def test_late_program_error_preserves_existing_json(self):
        source = self.folder / 'invalid program.yaml'
        profile = (viewer.ROOT / 'models/ur5e/profile.yaml').as_posix()
        source.write_text('schema_version: 1\nunits: SI\nrobot_profile: ' + profile + '\n'
                          'start: {joints: [0.2,-0.6,0.7,0.3,0.8,-0.2]}\nsteps:\n'
                          ' - {type: PTP, joints: [0.21,-0.6,0.7,0.3,0.8,-0.2]}\n'
                          ' - {type: PTP, joints: [100,0,0,0,0,0]}\n')
        output = self.folder / 'prior.json'
        output.write_text('prior valid export')
        result = subprocess.run([str(EXPORTER), str(source), str(output)], capture_output=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'step 2', result.stderr)
        self.assertEqual(output.read_text(), 'prior valid export')
        self.assertFalse(list(self.folder.glob('prior.json.tmp-*')))

    def test_native_budget_preserves_existing_json(self):
        output = self.folder / 'budget.json'
        output.write_text('prior valid export')
        result = subprocess.run([str(EXPORTER), str(viewer.TOOL_DIR / 'demo/ur5e_visual_demo.yaml'), str(output), '10', '1'],
                                capture_output=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output.read_text(), 'prior valid export')
        self.assertFalse(list(self.folder.glob('budget.json.tmp-*')))

    def test_source_cannot_be_overwritten(self):
        source = self.folder / 'program.json'
        profile = (viewer.ROOT / 'models/ur5e/profile.yaml').as_posix()
        content = ('schema_version: 1\nunits: SI\nrobot_profile: ' + profile + '\n'
                   'start: {joints: [0.2,-0.6,0.7,0.3,0.8,-0.2]}\n'
                   'steps: [{type: WAIT, seconds: 0.01}]\n')
        source.write_text(content)
        result = subprocess.run([str(EXPORTER), str(source), str(source)], capture_output=True, timeout=20)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'Input and output must differ', result.stderr)
        self.assertEqual(source.read_text(), content)
        with self.assertRaisesRegex(ValueError, 'html extension'):
            viewer.export_and_render(source, EXPORTER, source)
        same_html = self.folder / 'program.html'
        same_html.write_text(content)
        with self.assertRaisesRegex(ValueError, 'must differ'):
            viewer.export_and_render(same_html, EXPORTER, same_html)
        self.assertEqual(same_html.read_text(), content)

    def test_embedded_labels_remain_inert_json(self):
        scene = copy.deepcopy(self.full)
        malicious = '</script><script>globalThis.unexpectedCode=true</script>\u2028&'
        scene['robot']['name'] = malicious
        scene['program']['segments'][0]['source_file'] = malicious
        rendered = viewer.render_html(scene)
        self.assertNotIn(malicious, rendered)
        parser = ScriptCounter()
        parser.feed(rendered)
        self.assertEqual(parser.count, 3)
        payload = re.search(r'<script id="motion-data" type="application/json">(.*?)</script>', rendered, re.S)[1]
        self.assertEqual(json.loads(payload)['robot']['name'], malicious)
        self.assertIn('\\u003c', payload)

    def test_invalid_metadata_and_json_rejected(self):
        for mutate in [lambda data: data['robot']['joints'][0].update(type='floating'),
                       lambda data: data['robot']['limits'].update(lower=[0]),
                       lambda data: data['program'].update(duration_s=1),
                       lambda data: data['robot']['base_pose'].update(orientation=[0,0,0,0])]:
            data = copy.deepcopy(self.exported)
            mutate(data)
            with self.assertRaises(ValueError): viewer.prepare_data(data)
        file = self.folder / 'duplicate.json'
        file.write_text('{"schema_version":1,"schema_version":1}')
        with self.assertRaises(ValueError): viewer.load_export(file)
        file.write_text('{"value":NaN}')
        with self.assertRaises(ValueError): viewer.load_export(file)

    def test_visual_selection_is_trusted_and_skeleton_export_is_self_contained(self):
        self.assertEqual(len(self.full['visual_model']['links']), 7)
        # Imported display payloads never supply executable/foreign geometry.
        exported = copy.deepcopy(self.exported)
        exported['visual_model'] = {'model': 'untrusted external asset'}
        for joint in exported['robot']['joints']:
            joint.pop('parent_link', None)
            joint.pop('child_link', None)
        legacy = viewer.prepare_data(exported, 40)
        self.assertNotIn('visual_model', legacy)
        skeleton = viewer.prepare_data(self.exported, 40, visuals='none')
        self.assertNotIn('visual_model', skeleton)
        html = viewer.render_html(skeleton)
        self.assertNotIn('THREE.WebGLRenderer', html)
        self.assertNotIn('base.dae', html)
        self.assertNotIn('/*__THREE__*/', html)
        self.assertNotIn('/*__MESH_RENDERER__*/', html)

    def test_cycle_grid_one_and_twenty_ms(self):
        # A shorter already admitted reference exercises the two extreme grids.
        source = viewer.ROOT / 'programs/examples/ur5e_demo.yaml'
        for cycle in (1, 20):
            file = self.folder / f'cycle-{cycle}.json'
            subprocess.run([str(EXPORTER), str(source), str(file), str(cycle)], check=True,
                           capture_output=True, timeout=60)
            data = viewer.prepare_data(viewer.load_export(file), 300)
            self.assertEqual(data['program']['cycle_s'], cycle * .001)
            self.assertFalse(data['sample_limit_violations'])

    @unittest.skipUnless(shutil.which('node'), 'Node optional: browser renderer comparison')
    def test_renderer_matches_three_reference_models_and_mixed_chain(self):
        files = [self.demo_file]
        for name in ('ur5e', 'abb_irb120', 'fanuc_lrmate200id'):
            file = self.folder / (name + '.json')
            subprocess.run([str(EXPORTER), str(viewer.ROOT / 'programs/examples' / (name + '_demo.yaml')), str(file)],
                           check=True, capture_output=True, timeout=60)
            files.append(file)
        fixture = self.folder / 'mixed'
        fixture.mkdir()
        (fixture / 'robot.urdf').write_text('''<robot name="mixed"><link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="tip"/>
<joint name="fixed" type="fixed"><parent link="base"/><child link="a"/><origin xyz="0.1 -0.2 0.05" rpy="0.2 -0.1 0.4"/></joint>
<joint name="r" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0.2 0.1 0.3" rpy="0.3 0.2 -0.1"/><axis xyz="1 2 3"/><limit lower="-2" upper="2" velocity="0.3"/></joint>
<joint name="p" type="prismatic"><parent link="b"/><child link="c"/><origin xyz="0.1 0 0.05" rpy="-0.2 0.1 0.2"/><axis xyz="1 1 0"/><limit lower="-0.2" upper="0.2" velocity="0.05"/></joint>
<joint name="w" type="revolute"><parent link="c"/><child link="tip"/><origin xyz="0.1 0.2 0" rpy="0.1 -0.3 0.2"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" velocity="0.3"/></joint></robot>''')
        (fixture / 'profile.yaml').write_text('''schema_version: 1
units: SI
robot_name: mixed
base_link: base
tip_link: tip
base_pose: {xyz: [0.2,0.1,0.3], rpy: [0.2,-0.1,0.3]}
tool_offset: {xyz: [0.05,0.02,0.01], rpy: [0.3,-0.2,0.1]}
joints:
  r: {max_acceleration: 1, max_jerk: 5}
  p: {max_acceleration: 0.5, max_jerk: 1}
  w: {max_acceleration: 1, max_jerk: 5}
tcp_limits: {max_linear_velocity: 1, max_linear_acceleration: 2, max_linear_jerk: 10, max_angular_velocity: 1, max_angular_acceleration: 2, max_angular_jerk: 10}
''')
        (fixture / 'program.yaml').write_text('schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\n'
                                             'start: {joints: [0.1,0.02,-0.2]}\nsteps: [{type: PTP, joints: [0.2,0.04,-0.1]}]')
        file = fixture / 'scene.json'
        subprocess.run([str(EXPORTER), str(fixture / 'program.yaml'), str(file)], check=True, capture_output=True, timeout=60)
        files.append(file)
        mixed = viewer.prepare_data(viewer.load_export(file))
        self.assertFalse(mixed['sample_limit_violations'])
        result = subprocess.run(['node', str(viewer.TOOL_DIR / 'tests/check_renderer_fk.js'), *map(str, files)],
                                check=True, capture_output=True, text=True, timeout=30)
        report = json.loads(result.stdout)
        self.assertGreater(report['cases'], 3000)
        self.assertLess(report['maxPosition'], 2e-8)
        self.assertLess(report['maxRotation'], 2e-8)
        print('Renderer FK comparison:', report)


if __name__ == '__main__':
    unittest.main()
