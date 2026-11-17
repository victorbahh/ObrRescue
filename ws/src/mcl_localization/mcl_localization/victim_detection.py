import math


def find_candidates(points, occupancy_map, separation=0.035, wall_margin=0.04):
    groups = []
    group = []
    for point in points + [None]:
        eligible = False
        if point is not None:
            cell = occupancy_map.cell(point[0], point[1])
            eligible = (cell is not None and occupancy_map.data[cell] == 0 and
                        occupancy_map.distances[cell] > wall_margin)
        if not eligible or (group and math.dist(point, group[-1]) > separation):
            if len(group) >= 2:
                groups.append(group)
            group = []
        if eligible:
            group.append(point)
    candidates = []
    for group in groups:
        diameter = max(math.dist(first, last) for first in group for last in group)
        if 0.008 <= diameter <= 0.075:
            candidates.append((sum(point[0] for point in group) / len(group),
                               sum(point[1] for point in group) / len(group)))
    return candidates


class CandidateTracker:
    def __init__(self, association_distance=0.07, expiration=5.0, min_observations=3, max_victims=3):
        self.association_distance = association_distance
        self.expiration = expiration
        self.min_observations = min_observations
        self.max_victims = max_victims
        self.tracks = []
        self.next_id = 0

    def update(self, candidates, timestamp):
        self.tracks = [track for track in self.tracks if 0 <= timestamp - track['time'] <= self.expiration]
        used = set()
        for x, y in candidates:
            matches = [(math.hypot(track['x'] - x, track['y'] - y), index)
                       for index, track in enumerate(self.tracks) if index not in used]
            distance, index = min(matches, default=(math.inf, -1))
            if distance <= self.association_distance:
                track = self.tracks[index]
                track['x'] = 0.7 * track['x'] + 0.3 * x
                track['y'] = 0.7 * track['y'] + 0.3 * y
                track['time'] = timestamp
                track['count'] += 1
                used.add(index)
            else:
                self.tracks.append({'id': self.next_id, 'x': x, 'y': y,
                                    'time': timestamp, 'count': 1})
                self.next_id += 1
                used.add(len(self.tracks) - 1)
        confirmed = [track for track in self.tracks if track['count'] >= self.min_observations]
        confirmed.sort(key=lambda track: (-track['count'], -track['time'], track['id']))
        return confirmed[:self.max_victims]