#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Build colcon workspace if needed (source is bind-mounted)
WS=/workspace/simulation/diffphys_ws
if [ -f "$WS/src/diffphys_local_planner/package.xml" ]; then
    if [ ! -f "$WS/install/setup.bash" ] || \
       [ "$WS/src/diffphys_local_planner/package.xml" -nt "$WS/install/setup.bash" ]; then
        echo "=== Building colcon workspace ==="
        cd "$WS" && colcon build --symlink-install --packages-select \
            cmake_utils pose_utils uav_utils \
            quadrotor_msgs traj_utils \
            plan_env path_searching bspline_opt drone_detect ego_planner \
            map_generator local_sensing \
            so3_control so3_quadrotor_simulator \
            odom_visualization poscmd_2_odom \
            diffphys_local_planner \
            2>&1
        echo "=== Build done ==="
    fi
    source "$WS/install/setup.bash"
fi

exec "$@"
