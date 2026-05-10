#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /workspace/simulation/diffphys_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sleep 10

echo "=== Sending goal ==="
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world}, pose: {position: {x: 15.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"

sleep 3

echo "=== Running validator (60s) ==="
python3 /workspace/simulation/diffphys_ws/install/diffphys_local_planner/lib/diffphys_local_planner/validator 60 15.0 0.0 1.0
echo "=== Validator done ==="
