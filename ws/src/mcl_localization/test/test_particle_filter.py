import math
import random
import unittest

from mcl_localization.particle_filter import OccupancyMap, ParticleFilter


class ParticleFilterTest(unittest.TestCase):
    def setUp(self):
        data = [0] * 100
        for row in range(10):
            data[row * 10 + 8] = 100
        self.grid = OccupancyMap(10, 10, 0.1, (0.0, 0.0, 0.0), data)

    def test_scan_prefers_particle_matching_wall(self):
        particle_filter = ParticleFilter(self.grid, 2, random.Random(1))
        particle_filter.particles = [[0.35, 0.45, 0.0, 0.5], [0.15, 0.45, 0.0, 0.5]]
        particle_filter.update([(0.0, 0.5)], (0.0, 0.0, 0.0))
        self.assertGreater(particle_filter.particles[0][3], 0.95)
        self.assertAlmostEqual(sum(particle[3] for particle in particle_filter.particles), 1.0)

    def test_motion_uses_previous_odom_heading(self):
        particle_filter = ParticleFilter(self.grid, 1, random.Random(1))
        particle_filter.particles = [[0.0, 0.0, math.pi / 2, 1.0]]
        particle_filter.rng.gauss = lambda mean, deviation: mean
        particle_filter.move((0.0, 0.0, math.pi / 2), (0.0, 0.1, math.pi / 2))
        self.assertAlmostEqual(particle_filter.particles[0][0], 0.0)
        self.assertAlmostEqual(particle_filter.particles[0][1], 0.1)

    def test_rotated_map_origin(self):
        grid = OccupancyMap(2, 2, 1.0, (1.0, 2.0, math.pi / 2), [0, 100, 0, 0])
        self.assertEqual(grid.cell(0.5, 3.5), 1)

    def test_resampling_keeps_count_and_normalizes_weights(self):
        particle_filter = ParticleFilter(self.grid, 4, random.Random(3))
        particle_filter.particles = [[0.35, 0.45, 0.0, 0.97], [0.1, 0.1, 0.0, 0.01],
                                     [0.2, 0.1, 0.0, 0.01], [0.3, 0.1, 0.0, 0.01]]
        particle_filter.resample_if_needed()
        self.assertEqual(len(particle_filter.particles), 4)
        self.assertTrue(all(particle[3] == 0.25 for particle in particle_filter.particles))
        self.assertTrue(all(particle[0] == 0.35 for particle in particle_filter.particles))

    def test_empty_scan_does_not_change_weights(self):
        particle_filter = ParticleFilter(self.grid, 2, random.Random(1))
        particle_filter.initialize((0.2, 0.2, 0.0), (0.0, 0.0, 0.0))
        self.assertFalse(particle_filter.update([], (0.0, 0.0, 0.0)))
        self.assertEqual([particle[3] for particle in particle_filter.particles], [0.5, 0.5])


if __name__ == '__main__':
    unittest.main()