import heapq
import math
import random


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OccupancyMap:
    def __init__(self, width, height, resolution, origin, data):
        if width <= 0 or height <= 0 or resolution <= 0 or len(data) != width * height:
            raise ValueError('Invalid occupancy grid')
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.data = list(data)
        self.distances = [math.inf] * len(data)
        queue = []
        for index, value in enumerate(self.data):
            if value >= 65:
                self.distances[index] = 0.0
                heapq.heappush(queue, (0.0, index))
        if not queue:
            raise ValueError('Map has no occupied cells')
        neighbors = [(dx, dy, math.hypot(dx, dy) * resolution)
                     for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy]
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != self.distances[index]:
                continue
            cell_x, cell_y = index % width, index // width
            for dx, dy, step in neighbors:
                next_x, next_y = cell_x + dx, cell_y + dy
                if 0 <= next_x < width and 0 <= next_y < height:
                    next_index = next_y * width + next_x
                    next_distance = distance + step
                    if next_distance < self.distances[next_index]:
                        self.distances[next_index] = next_distance
                        heapq.heappush(queue, (next_distance, next_index))

    def cell(self, x, y):
        origin_x, origin_y, origin_yaw = self.origin
        dx, dy = x - origin_x, y - origin_y
        cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
        cell_x = math.floor((cosine * dx + sine * dy) / self.resolution)
        cell_y = math.floor((-sine * dx + cosine * dy) / self.resolution)
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            return cell_y * self.width + cell_x
        return None


class ParticleFilter:
    def __init__(self, occupancy_map, count=300, rng=None):
        if count <= 0:
            raise ValueError('Particle count must be positive')
        self.map = occupancy_map
        self.count = count
        self.rng = rng or random.Random()
        self.particles = []

    def initialize(self, pose, standard_deviations):
        self.particles = [
            [self.rng.gauss(pose[0], standard_deviations[0]),
             self.rng.gauss(pose[1], standard_deviations[1]),
             wrap(self.rng.gauss(pose[2], standard_deviations[2])), 1.0 / self.count]
            for _ in range(self.count)
        ]

    def move(self, previous, current):
        dx, dy = current[0] - previous[0], current[1] - previous[1]
        cosine, sine = math.cos(previous[2]), math.sin(previous[2])
        forward = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        rotation = wrap(current[2] - previous[2])
        distance = math.hypot(forward, lateral)
        if distance < 1e-6 and abs(rotation) < 1e-6:
            return
        translation_noise = 0.03 * distance + 0.002 * abs(rotation)
        rotation_noise = 0.05 * abs(rotation) + 0.03 * distance
        for particle in self.particles:
            noisy_forward = forward + self.rng.gauss(0.0, translation_noise)
            noisy_lateral = lateral + self.rng.gauss(0.0, translation_noise)
            cosine, sine = math.cos(particle[2]), math.sin(particle[2])
            particle[0] += cosine * noisy_forward - sine * noisy_lateral
            particle[1] += sine * noisy_forward + cosine * noisy_lateral
            particle[2] = wrap(particle[2] + rotation + self.rng.gauss(0.0, rotation_noise))

    def update(self, beams, laser_pose, sigma=0.05):
        if not self.particles or not beams:
            return False
        scores = []
        for x, y, yaw, weight in self.particles:
            cosine, sine = math.cos(yaw), math.sin(yaw)
            laser_x = x + cosine * laser_pose[0] - sine * laser_pose[1]
            laser_y = y + sine * laser_pose[0] + cosine * laser_pose[1]
            score = math.log(max(weight, 1e-300))
            for angle, distance in beams:
                direction = yaw + laser_pose[2] + angle
                index = self.map.cell(laser_x + distance * math.cos(direction),
                                      laser_y + distance * math.sin(direction))
                probability = 0.02
                if index is not None and self.map.data[index] >= 0:
                    error = self.map.distances[index]
                    probability += 0.98 * math.exp(-0.5 * (error / sigma) ** 2)
                score += math.log(probability)
            scores.append(score)
        maximum = max(scores)
        weights = [math.exp(score - maximum) for score in scores]
        total = sum(weights)
        for particle, weight in zip(self.particles, weights):
            particle[3] = weight / total
        return True

    def estimate(self):
        x = sum(particle[0] * particle[3] for particle in self.particles)
        y = sum(particle[1] * particle[3] for particle in self.particles)
        yaw = math.atan2(sum(math.sin(particle[2]) * particle[3] for particle in self.particles),
                         sum(math.cos(particle[2]) * particle[3] for particle in self.particles))
        variances = [sum(particle[3] * (particle[0] - x) ** 2 for particle in self.particles),
                     sum(particle[3] * (particle[1] - y) ** 2 for particle in self.particles),
                     sum(particle[3] * wrap(particle[2] - yaw) ** 2 for particle in self.particles)]
        return (x, y, yaw), variances

    def resample_if_needed(self):
        effective = 1.0 / sum(particle[3] ** 2 for particle in self.particles)
        if effective >= self.count / 2:
            return
        position = self.rng.random() / self.count
        index = 0
        cumulative = self.particles[0][3]
        resampled = []
        for sample in range(self.count):
            target = position + sample / self.count
            while target > cumulative and index < self.count - 1:
                index += 1
                cumulative += self.particles[index][3]
            resampled.append(self.particles[index][:3] + [1.0 / self.count])
        self.particles = resampled