"""LIN contract through the real native exporter, CSV and offline viewer data."""
import csv
import io
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

import create_viewer as viewer
from test_program_editor import EXPORTER, request_for
from editor_server import EditorError, SimulationService


def rotation_distance(a, b):
    w, x, y, z = [v / math.hypot(*a) for v in a]
    t, u, v, s = [v / math.hypot(*b) for v in b]
    return 2 * math.atan2(math.hypot(w*u-x*t-y*s+z*v, w*v+x*s-y*t-z*u, w*s-x*v+y*u-z*t),
                         abs(w*t+x*u+y*v+z*s))


class LinearOrientationTests(unittest.TestCase):
    def test_actual_tcp_orientation_at_every_lin_tick_and_viewer_metadata(self):
        source = viewer.ROOT / "programs/examples/ur5e_lin_keep_orientation.yaml"
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "scene.json"
            subprocess.run([str(EXPORTER), str(source), str(output), "10"], check=True, capture_output=True)
            scene = viewer.load_export(output)
        rows = list(csv.DictReader(io.StringIO(scene["trace_csv"])))
        segment = next(item for item in scene['program']['segments'] if item['operation'] == 'LIN')
        self.assertEqual(segment['orientation_mode'], 'keep')
        line = [row for row in rows if int(row['segment']) == segment['index']]
        # Include the preceding boundary tick, which establishes the actual start.
        begin = rows[int(line[0]['tick'])]
        expected = [float(begin['q'+axis]) for axis in 'wxyz']
        maximum = max(rotation_distance(expected, [float(row['q'+axis]) for axis in 'wxyz']) for row in line)
        self.assertLess(maximum, 5e-4)
        data = viewer.prepare_data(scene, max_frames=100, visuals='none')
        self.assertFalse(data['sample_limit_violations'])
        self.assertEqual(next(item for item in data['program']['segments'] if item['operation']=='LIN')['orientation_mode'], 'keep')
        print(f'UR5e LIN: {len(line)} ticks, maximum orientation change {math.degrees(maximum):.9g} deg')

    def test_user_far_goal_rejected_and_explicit_rotation_remains_available(self):
        source = (viewer.ROOT / 'programs/examples/ur5e_lin_keep_orientation.yaml').read_text(encoding='utf-8')
        source = source.replace('../../models/ur5e/profile_pick_place_simulation.yaml',
                                'models/ur5e/profile_pick_place_simulation.yaml')
        source = source.replace('[-0.47390467854451826, 0.62398337528833302, 0.24366518271764495]',
                                '[0.7444144539141021, 0.3622280684167766, 0.24366518271764495]')
        source = source.replace('[-0.36982586929249855, -0.43651321706789414, 0.67017312781802196, 0.47281393430999302]',
                                '[0.1404804311152778, 0.25362367811777614, 0.7585188460561569, 0.5836004099975534]')
        service = SimulationService(EXPORTER)
        try:
            example = service.example('ur5e_lin')
            self.assertIn('robot_profile: models/ur5e/profile_pick_place_simulation.yaml', example['source'])
            self.assertIn('joints: [2,', example['source'])
            with self.assertRaises(EditorError) as caught:
                service.simulate(request_for(source))
            self.assertEqual(caught.exception.diagnostic['step'], 3)
            self.assertIn('constant TCP orientation', caught.exception.diagnostic['message'])
            self.assertFalse(service._results)
            self.assertFalse(list(service.folder.iterdir()))
            rotating = source.replace('type: LIN, target: work, speed: 0.6',
                                      'type: LIN, target: work, speed: 0.6, orientation_mode: interpolate')
            result = service.simulate(request_for(rotating))
            self.assertTrue(result['commands'] > 0)
            html, _ = service.result_file(result['id'])
            self.assertIn(b'"orientation_mode":"interpolate"', html)
        finally:
            service.close()
