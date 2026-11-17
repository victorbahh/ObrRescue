from collections import deque
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from mcl_localization.world_map import world_grid


class WorldMapTest(unittest.TestCase):
    def test_room_features_and_dynamic_victims(self):
        sdf = Path(__file__).resolve().parents[2] / 'area_resgate_obr.sdf'
        width, height, resolution, origin, cells = world_grid(sdf)

        def cell(x, y):
            column = int((x - origin[0]) / resolution)
            row = int((y - origin[1]) / resolution)
            self.assertTrue(0 <= column < width and 0 <= row < height)
            return cells[row * width + column]

        self.assertEqual(cell(-0.65, -0.325), 0)
        self.assertEqual(cell(0.65, 0.325), 0)
        self.assertEqual(cell(0.0, 0.46), 100)
        self.assertEqual(cell(-0.30, 0.30), 100)
        self.assertEqual(cell(-0.10, 0.10), 0)
        self.assertEqual(cell(0.05, -0.15), 0)
        self.assertEqual(cell(0.15, 0.20), 0)

    def test_robot_fits_between_entrance_and_exit(self):
        sdf = Path(__file__).resolve().parents[2] / 'area_resgate_obr.sdf'
        width, height, resolution, origin, cells = world_grid(sdf)
        world = ET.parse(sdf)
        for victim in world.findall('.//model'):
            if not victim.get('name', '').startswith('victim_'):
                continue
            x, y = (float(value) for value in victim.findtext('pose').split()[:2])
            radius = float(victim.findtext('.//collision/geometry/sphere/radius'))
            for row in range(height):
                for column in range(width):
                    cell_x = origin[0] + (column + 0.5) * resolution
                    cell_y = origin[1] + (row + 0.5) * resolution
                    if (cell_x - x) ** 2 + (cell_y - y) ** 2 <= radius ** 2:
                        cells[row * width + column] = 100
        offsets = [(dx, dy) for dx in range(-6, 7) for dy in range(-6, 7)
                   if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= 0.052 ** 2]

        def grid_position(x, y):
            return int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)

        def traversable(column, row):
            return all(0 <= column + dx < width and 0 <= row + dy < height and
                       cells[(row + dy) * width + column + dx] == 0 for dx, dy in offsets)

        start = grid_position(-0.65, -0.325)
        goal = grid_position(0.65, 0.325)
        pending = deque([start])
        visited = {start}
        while pending:
            column, row = pending.popleft()
            if (column, row) == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (column + dx, row + dy)
                if neighbor not in visited and traversable(*neighbor):
                    visited.add(neighbor)
                    pending.append(neighbor)
        self.assertIn(goal, visited)

    def test_navigation_retains_collision_protection(self):
        package = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package / 'config' / 'navigation.yaml').read_text())
        control = config['controller_server']['ros__parameters']['FollowPath']
        monitor = config['collision_monitor']['ros__parameters']
        zone = monitor['StopZone']
        robot = ET.parse(package.parent / 'hero_sim' / 'urdf' / 'hero_sim.urdf.xacro')
        wheel = robot.find('.//{http://www.ros.org/wiki/xacro}macro[@name="wheel"]')
        wheel_geometry = wheel.find('./link/collision/geometry/cylinder')
        wheel_side = float(robot.find('.//{http://www.ros.org/wiki/xacro}wheel[@side="left"]').get('y'))
        wheel_radius = float(wheel_geometry.get('radius'))
        wheel_half_width = float(wheel_geometry.get('length')) / 2
        minimum_radius = math.hypot(wheel_radius, wheel_side + wheel_half_width)
        configured_radius = config['global_costmap']['global_costmap']['ros__parameters']['robot_radius']
        self.assertGreaterEqual(configured_radius, minimum_radius)
        self.assertLess(control['min_vel_x'], 0.0)
        self.assertLessEqual(control['max_vel_x'], 0.04)
        self.assertGreater(monitor['source_timeout'], 0.0)
        self.assertEqual(zone['type'], 'velocity_polygon')
        self.assertEqual(zone['action_type'], 'stop')
        for velocity in (control['min_vel_x'], 0.0, control['max_vel_x']):
            self.assertTrue(any(zone[name]['linear_min'] <= velocity <= zone[name]['linear_max']
                                and zone[name]['theta_min'] <= -control['max_vel_theta']
                                and zone[name]['theta_max'] >= control['max_vel_theta']
                                for name in zone['velocity_polygons']))
        for name in zone['velocity_polygons']:
            points = yaml.safe_load(zone[name]['points'])
            self.assertLessEqual(min(point[0] for point in points), -configured_radius)
            self.assertGreaterEqual(max(point[0] for point in points), configured_radius)
            self.assertLessEqual(min(point[1] for point in points), -configured_radius)
            self.assertGreaterEqual(max(point[1] for point in points), configured_radius)
        for layer in ('global_costmap', 'local_costmap'):
            costmap = config[layer][layer]['ros__parameters']
            scan = costmap['obstacle_layer']['scan']
            self.assertTrue(scan['marking'])
            self.assertLessEqual(scan['min_obstacle_height'], 0.032)
            self.assertGreaterEqual(scan['max_obstacle_height'], 0.032)
            self.assertEqual(costmap['robot_radius'], configured_radius)
            self.assertGreater(costmap['inflation_layer']['inflation_radius'], 0.065)
        tree = ET.parse(package / 'config' / 'navigate.xml')
        recovery = tree.find('.//RecoveryNode')
        self.assertIsNotNone(recovery)
        self.assertGreater(int(recovery.get('number_of_retries')), 0)


if __name__ == '__main__':
    unittest.main()
