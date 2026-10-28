#!/usr/bin/env python3
"""Generate tag36h11 textures and Gazebo models from a tag map.

Each tag is a textured plane whose +z face carries the marker, sized to the
tag's black-border edge length.
"""

import argparse
import os

import cv2
import numpy as np
import yaml

try:
    import cv2.aruco as aruco
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'cv2.aruco is required (install an OpenCV build with the contrib modules)'
    ) from exc


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>apriltag_{id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>AprilTag {family} id {id}</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="apriltag_{id}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{size} {size}</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://apriltag_{id}/materials/scripts</uri>
            <uri>model://apriltag_{id}/materials/textures</uri>
            <name>apriltag_{id}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MATERIAL = """material apriltag_{id}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture tag_{id}.png
        filtering none none none
      }}
    }}
  }}
}}
"""


def make_marker(dictionary, tag_id, pixels):
    if hasattr(aruco, 'generateImageMarker'):
        return aruco.generateImageMarker(dictionary, tag_id, pixels)
    img = np.zeros((pixels, pixels), dtype=np.uint8)
    aruco.drawMarker(dictionary, tag_id, pixels, img, 1)
    return img


def write_model(output_dir, tag_id, family, size, image):
    model_dir = os.path.join(output_dir, f'apriltag_{tag_id}')
    scripts_dir = os.path.join(model_dir, 'materials', 'scripts')
    textures_dir = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(scripts_dir, exist_ok=True)
    os.makedirs(textures_dir, exist_ok=True)

    cv2.imwrite(os.path.join(textures_dir, f'tag_{tag_id}.png'), image)
    with open(os.path.join(model_dir, 'model.config'), 'w') as handle:
        handle.write(MODEL_CONFIG.format(id=tag_id, family=family))
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as handle:
        handle.write(MODEL_SDF.format(id=tag_id, size=size))
    with open(os.path.join(scripts_dir, f'apriltag_{tag_id}.material'), 'w') as handle:
        handle.write(MATERIAL.format(id=tag_id))


def load_tags(tag_map_path):
    with open(tag_map_path) as handle:
        data = yaml.safe_load(handle)
    default_size = data.get('default_size', 0.4)
    tags = []
    for entry in data.get('tags', []):
        tags.append((int(entry['id']), float(entry.get('size', default_size))))
    return tags


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag-map', help='tags.yaml to read ids and sizes from')
    parser.add_argument('--ids', type=int, nargs='*', help='explicit tag ids')
    parser.add_argument('--size', type=float, default=0.4, help='edge length (m)')
    parser.add_argument('--pixels', type=int, default=600, help='texture resolution')
    parser.add_argument('--family', default='tag36h11')
    parser.add_argument('--output', required=True, help='models output directory')
    args = parser.parse_args(argv)

    families = {
        'tag36h11': aruco.DICT_APRILTAG_36h11,
        'tag25h9': aruco.DICT_APRILTAG_25h9,
        'tag16h5': aruco.DICT_APRILTAG_16h5,
    }
    dictionary = aruco.getPredefinedDictionary(families[args.family])

    if args.tag_map:
        tags = load_tags(args.tag_map)
    elif args.ids:
        tags = [(i, args.size) for i in args.ids]
    else:
        parser.error('provide either --tag-map or --ids')

    for tag_id, size in tags:
        marker = make_marker(dictionary, tag_id, args.pixels)
        write_model(args.output, tag_id, args.family, size, marker)
        print(f'wrote model apriltag_{tag_id} (size {size} m)')


if __name__ == '__main__':
    main()
