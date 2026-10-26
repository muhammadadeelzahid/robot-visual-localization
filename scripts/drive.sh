#!/bin/bash
# Teleoperate the simulated robot. Run in a terminal with keyboard focus.
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
