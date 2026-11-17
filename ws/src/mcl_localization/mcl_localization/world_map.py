import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def pose(element):
    value = element.find('pose')
    if value is None:
        return (0.0, 0.0, 0.0, 0.0)
    if value.get('relative_to'):
        raise ValueError('Poses with relative_to are not supported')
    parts = [float(part) for part in value.text.split()]
    if len(parts) != 6 or abs(parts[3]) > 1e-8 or abs(parts[4]) > 1e-8:
        raise ValueError('Only planar poses are supported')
    return parts[0], parts[1], parts[2], parts[5]


def compose(parent, child):
    cosine, sine = math.cos(parent[3]), math.sin(parent[3])
    return (parent[0] + cosine * child[0] - sine * child[1],
            parent[1] + sine * child[0] + cosine * child[1],
            parent[2] + child[2], parent[3] + child[3])


def contains(box, x, y):
    center, size = box
    dx, dy = x - center[0], y - center[1]
    cosine, sine = math.cos(center[3]), math.sin(center[3])
    return (abs(cosine * dx + sine * dy) <= size[0] / 2 and
            abs(-sine * dx + cosine * dy) <= size[1] / 2)


def world_grid(sdf_file, resolution=0.01, laser_height=0.032):
    if resolution <= 0:
        raise ValueError('Resolution must be positive')
    world = ET.parse(sdf_file).getroot().find('world')
    if world is None:
        raise ValueError('SDF must contain a world')
    floors, obstacles = [], []
    for model in world.findall('model'):
        if model.get('name', '').startswith('victim_'):
            continue
        model_pose = pose(model)
        for link in model.findall('link'):
            link_pose = compose(model_pose, pose(link))
            for collision in link.findall('collision'):
                box = collision.find('geometry/box/size')
                if box is None:
                    continue
                size = tuple(float(part) for part in box.text.split())
                if len(size) != 3 or min(size) <= 0:
                    raise ValueError('Invalid box size')
                center = compose(link_pose, pose(collision))
                if center[2] + size[2] / 2 <= 0.005:
                    floors.append((center, size))
                elif center[2] - size[2] / 2 <= laser_height <= center[2] + size[2] / 2:
                    obstacles.append((center, size))
    if not floors or not obstacles:
        raise ValueError('Could not find floor and obstacles in SDF')
    corners = []
    for center, size in floors:
        cosine, sine = math.cos(center[3]), math.sin(center[3])
        for dx in (-size[0] / 2, size[0] / 2):
            for dy in (-size[1] / 2, size[1] / 2):
                corners.append((center[0] + cosine * dx - sine * dy,
                                center[1] + sine * dx + cosine * dy))
    min_x = math.floor((min(point[0] for point in corners) - 0.1) / resolution) * resolution
    min_y = math.floor((min(point[1] for point in corners) - 0.1) / resolution) * resolution
    max_x = max(point[0] for point in corners) + 0.1
    max_y = max(point[1] for point in corners) + 0.1
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)
    cells = []
    for row in range(height):
        y = min_y + (row + 0.5) * resolution
        for column in range(width):
            x = min_x + (column + 0.5) * resolution
            if any(contains(box, x, y) for box in obstacles):
                cells.append(100)
            elif any(contains(box, x, y) for box in floors):
                cells.append(0)
            else:
                cells.append(-1)
    return width, height, resolution, (min_x, min_y), cells


def main():
    parser = argparse.ArgumentParser(description='Generate a world-aligned static occupancy map from the OBR SDF')
    parser.add_argument('sdf', type=Path)
    parser.add_argument('output', type=Path, help='Output prefix for .pgm and .yaml')
    arguments = parser.parse_args()
    width, height, resolution, origin, cells = world_grid(arguments.sdf)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    image = arguments.output.with_suffix('.pgm')
    pixels = bytes({-1: 205, 0: 254, 100: 0}[cells[row * width + col]]
                   for row in range(height - 1, -1, -1) for col in range(width))
    image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    arguments.output.with_suffix('.yaml').write_text(
        f'image: {image.name}\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


if __name__ == '__main__':
    main()