#!/usr/bin/env python3
"""Generate a consistent warehouse scenario from a single layout definition.

Emits the Gazebo world, a matching occupancy grid for navigation, and the tag
map. Keeping all three from one source avoids drift between the simulated
geometry, the planner's map, and the poses the estimator expects.
"""

import argparse
import math
import os

import numpy as np


# --- Layout (metres, map frame centred at the warehouse origin) --------------

HALF_X, HALF_Y = 10.0, 7.0          # inner wall half-extents
WALL_T = 0.2                        # wall thickness
WALL_H = 2.5

# Static box obstacles: name, centre x, y, size x, y, height.
OBSTACLES = [
    ('shelf_nw', -3.5, 3.0, 5.0, 1.0, 2.0),
    ('shelf_ne', 3.5, 3.0, 5.0, 1.0, 2.0),
    ('shelf_sw', -3.5, -3.0, 5.0, 1.0, 2.0),
    ('shelf_se', 3.5, -3.0, 5.0, 1.0, 2.0),
    ('pillar_n', 0.0, 5.2, 0.4, 0.4, 2.5),
    ('pillar_s', 0.0, -5.2, 0.4, 0.4, 2.5),
    ('pillar_w', -7.5, 0.0, 0.4, 0.4, 2.5),
    ('pillar_e', 7.5, 0.0, 0.4, 0.4, 2.5),
    ('pallet_1', -5.0, 0.0, 0.8, 1.2, 0.6),
    ('pallet_2', 5.0, 0.8, 1.2, 0.8, 0.6),
]

# Tags: id, size, centre x, y, z, outward normal (into the navigable space).
TAGS = [
    (0, 0.4, 9.8, 0.0, 1.0, (-1, 0, 0)),     # +x wall, faces aisle
    (1, 0.4, -9.8, 0.0, 1.0, (1, 0, 0)),     # -x wall
    (2, 0.4, 0.0, 6.8, 1.0, (0, -1, 0)),     # +y wall
    (3, 0.4, 0.0, -6.8, 1.0, (0, 1, 0)),     # -y wall
    (4, 0.3, 0.0, 4.95, 1.0, (0, -1, 0)),    # pillar_n, faces aisle
    (5, 0.3, 0.0, -4.95, 1.0, (0, 1, 0)),    # pillar_s
    (6, 0.3, -7.25, 0.0, 1.0, (1, 0, 0)),    # pillar_w
    (7, 0.3, 7.25, 0.0, 1.0, (-1, 0, 0)),    # pillar_e
]

MAP_RES = 0.05
MAP_MARGIN = 0.5


# --- Geometry helpers --------------------------------------------------------

def quat_from_matrix(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            w = (r[2, 1] - r[1, 2]) / s; x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s; z = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            w = (r[0, 2] - r[2, 0]) / s; x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s; z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            w = (r[1, 0] - r[0, 1]) / s; x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def tag_quaternion(normal):
    z = np.array(normal, float); z /= np.linalg.norm(z)
    up = np.array([0, 0, 1.0])
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:          # normal parallel to up
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


def quat_to_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# --- Emitters ----------------------------------------------------------------

def box_model(name, cx, cy, sx, sy, h):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {h/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>0.5 0.4 0.3 1</ambient><diffuse>0.6 0.5 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def wall_model():
    segs = [
        ('wall_px', HALF_X, 0.0, WALL_T, 2 * HALF_Y + WALL_T, WALL_H),
        ('wall_nx', -HALF_X, 0.0, WALL_T, 2 * HALF_Y + WALL_T, WALL_H),
        ('wall_py', 0.0, HALF_Y, 2 * HALF_X + WALL_T, WALL_T, WALL_H),
        ('wall_ny', 0.0, -HALF_Y, 2 * HALF_X + WALL_T, WALL_T, WALL_H),
    ]
    out = '    <model name="walls">\n      <static>true</static>\n      <link name="link">\n'
    for name, cx, cy, sx, sy, h in segs:
        out += f"""        <collision name="{name}_c"><pose>{cx} {cy} {h/2:.3f} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="{name}_v"><pose>{cx} {cy} {h/2:.3f} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
"""
    out += '      </link>\n    </model>\n'
    return out


def tag_includes():
    out = ''
    for tid, _size, cx, cy, cz, normal in TAGS:
        r, p, y = quat_to_rpy(tag_quaternion(normal))
        out += f"""    <include>
      <name>apriltag_{tid}</name>
      <uri>model://apriltag_{tid}</uri>
      <pose>{cx} {cy} {cz} {r:.6f} {p:.6f} {y:.6f}</pose>
    </include>
"""
    return out


def write_world(path):
    body = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.75 0.8 1</background></scene>

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>50.0</update_rate>
    </plugin>

{wall_model()}"""
    for name, cx, cy, sx, sy, h in OBSTACLES:
        body += box_model(name, cx, cy, sx, sy, h)
    body += tag_includes()
    body += "  </world>\n</sdf>\n"
    with open(path, 'w') as f:
        f.write(body)


def write_tags(path):
    lines = ['# Generated by generate_warehouse.py. Poses match warehouse.world.',
             'frame_id: map', 'default_size: 0.4', 'tags:']
    for tid, size, cx, cy, cz, normal in TAGS:
        q = tag_quaternion(normal)
        lines.append(f'  - id: {tid}')
        lines.append(f'    size: {size}')
        lines.append(f'    position: [{cx}, {cy}, {cz}]')
        lines.append('    orientation: [%.6f, %.6f, %.6f, %.6f]' % tuple(q))
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_map(pgm_path, yaml_path):
    w = int(round((2 * HALF_X + 2 * MAP_MARGIN) / MAP_RES))
    h = int(round((2 * HALF_Y + 2 * MAP_MARGIN) / MAP_RES))
    origin_x = -HALF_X - MAP_MARGIN
    origin_y = -HALF_Y - MAP_MARGIN
    grid = np.full((h, w), 254, dtype=np.uint8)   # free

    def fill(cx, cy, sx, sy):
        x0 = int((cx - sx / 2 - origin_x) / MAP_RES)
        x1 = int((cx + sx / 2 - origin_x) / MAP_RES)
        y0 = int((cy - sy / 2 - origin_y) / MAP_RES)
        y1 = int((cy + sy / 2 - origin_y) / MAP_RES)
        for py in range(max(0, y0), min(h, y1 + 1)):
            row = h - 1 - py                       # image row 0 = top (max y)
            grid[row, max(0, x0):min(w, x1 + 1)] = 0

    fill(HALF_X, 0.0, WALL_T, 2 * HALF_Y + WALL_T)
    fill(-HALF_X, 0.0, WALL_T, 2 * HALF_Y + WALL_T)
    fill(0.0, HALF_Y, 2 * HALF_X + WALL_T, WALL_T)
    fill(0.0, -HALF_Y, 2 * HALF_X + WALL_T, WALL_T)
    for _name, cx, cy, sx, sy, _h in OBSTACLES:
        fill(cx, cy, sx, sy)

    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(grid.tobytes())

    name = os.path.basename(pgm_path)
    with open(yaml_path, 'w') as f:
        f.write(
            f"image: {name}\nresolution: {MAP_RES}\n"
            f"origin: [{origin_x}, {origin_y}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True)
    parser.add_argument('--tags', required=True)
    parser.add_argument('--map-pgm', required=True)
    parser.add_argument('--map-yaml', required=True)
    args = parser.parse_args(argv)

    write_world(args.world)
    write_tags(args.tags)
    write_map(args.map_pgm, args.map_yaml)
    print(f'wrote {args.world}')
    print(f'wrote {args.tags}')
    print(f'wrote {args.map_pgm} + {args.map_yaml}')


if __name__ == '__main__':
    main()
