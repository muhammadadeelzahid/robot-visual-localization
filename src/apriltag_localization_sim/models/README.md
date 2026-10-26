# Tag models

Tag models are generated from the tag map so the simulated markers match the
poses and sizes the estimator expects:

```bash
ros2 run apriltag_localization_tools generate_apriltag_textures \
  --tag-map $(ros2 pkg prefix apriltag_localization_bringup)/share/apriltag_localization_bringup/config/tags.yaml \
  --output $(ros2 pkg prefix apriltag_localization_sim)/share/apriltag_localization_sim/models
```

Run this once after building and before launching the simulation.
