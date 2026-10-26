# robot-visual-localization

Camera-based 6-DoF localization for a mobile robot using AprilTags. A calibrated
camera detects fiducial markers, a pose estimator recovers the camera pose with
`solvePnP`, the pose is transformed into the world frame through a known tag map and
the camera-to-base extrinsic, and the result is fused with wheel odometry by an EKF to
drive the navigation stack.

## Packages

| Package | Role |
|---|---|
| `apriltag_detector` | AprilTag detection from image streams |
| `apriltag_pose_estimator` | PnP pose estimation and frame transforms |
| `apriltag_localization_bringup` | Launch files, configuration, RViz, tag maps, EKF |
| `apriltag_localization_sim` | Gazebo world, robot model, tag models |
| `apriltag_localization_tools` | Calibration, tag generation, accuracy analysis |

## Build

```bash
cd docker
docker compose build
docker compose run --rm dev
# inside the container:
colcon build --symlink-install
source install/setup.bash
```

## Run

Simulation:

```bash
ros2 launch apriltag_localization_bringup sim.launch.py
```

Physical robot:

```bash
ros2 launch apriltag_localization_bringup robot.launch.py camera:=usb
```

Both entry points share the same detection, pose-estimation and fusion pipeline; only
the image source and calibration differ.

## Camera calibration

```bash
ros2 run apriltag_localization_tools calibrate_camera \
  --images 'calib/*.png' --rows 7 --cols 9 --square 0.025 \
  --output src/apriltag_localization_bringup/config/camera_intrinsics.yaml
```

## Tag map

Tag identifiers, sizes and world poses are defined in
`apriltag_localization_bringup/config/tags.yaml`. The same file drives the estimator,
the RViz markers and the simulated world.

## License

MIT. See [LICENSE](LICENSE).
