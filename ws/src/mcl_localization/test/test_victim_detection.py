import ast
from collections import deque
import inspect
from pathlib import Path
import unittest

from mcl_localization.particle_filter import OccupancyMap
from mcl_localization.victim_detection import CandidateTracker, find_candidates


class VictimDetectionTest(unittest.TestCase):
    def setUp(self):
        cells = [0] * 10000
        for row in range(100):
            cells[row * 100 + 80] = 100
        self.grid = OccupancyMap(100, 100, 0.01, (0.0, 0.0, 0.0), cells)

    def test_candidate_away_from_known_wall(self):
        points = [(0.30, 0.40), (0.30, 0.41), (0.30, 0.42), None,
                  (0.79, 0.40), (0.79, 0.41), (0.79, 0.42)]
        candidates = find_candidates(points, self.grid)
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0][0], 0.30)

    def test_tracks_require_repeated_observations_and_expire(self):
        tracker = CandidateTracker()
        self.assertEqual(tracker.update([(0.2, 0.2)], 1.0), [])
        self.assertEqual(tracker.update([(0.21, 0.2)], 2.0), [])
        tracks = tracker.update([(0.21, 0.2)], 3.0)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracker.update([], 9.0), [])

    def test_only_three_most_observed_candidates_are_returned(self):
        tracker = CandidateTracker()
        candidates = [(0.1, 0.1), (0.3, 0.1), (0.5, 0.1), (0.7, 0.1)]
        for timestamp in range(3):
            tracker.update(candidates, float(timestamp))
        tracks = tracker.update(candidates[1:], 3.0)
        self.assertEqual(len(tracks), 3)
        self.assertEqual({track['id'] for track in tracks}, {1, 2, 3})

    def test_scan_callback_has_rclpy_compatible_signature(self):
        source = (Path(__file__).resolve().parents[1] / 'mcl_localization' / 'victim_node.py').read_text()
        tree = ast.parse(source)
        node_class = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                          and node.name == 'VictimNode')
        callback = next(node for node in node_class.body if isinstance(node, ast.FunctionDef)
                        and node.name == 'scan_callback')
        namespace = {}
        exec(compile(ast.Module(body=[callback], type_ignores=[]), 'victim_node.py', 'exec'), namespace)
        node = type('TestNode', (), {'scans': deque()})()
        bound_callback = namespace['scan_callback'].__get__(node)
        scan = object()
        inspect.signature(bound_callback).bind(scan)
        bound_callback(scan)
        self.assertIs(node.scans.pop(), scan)
        self.assertIn('self.scan_callback, qos_profile_sensor_data', source)


if __name__ == '__main__':
    unittest.main()