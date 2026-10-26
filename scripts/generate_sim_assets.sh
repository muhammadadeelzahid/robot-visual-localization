#!/bin/bash
# Generate the simulated tag models from the tag map. Run once after building.
set -e

BRINGUP_SHARE=$(ros2 pkg prefix apriltag_localization_bringup)/share/apriltag_localization_bringup
SIM_SHARE=$(ros2 pkg prefix apriltag_localization_sim)/share/apriltag_localization_sim

ros2 run apriltag_localization_tools generate_apriltag_textures \
  --tag-map "${BRINGUP_SHARE}/config/tags.yaml" \
  --output "${SIM_SHARE}/models"
