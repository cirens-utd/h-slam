#!/bin/bash

set -e

# Source workspace
source ~/.bashrc
source ~/ssv2_ws/devel/setup.bash

echo "Building ORB_SLAM2..."
cd ~/ssv2_ws/src/semantic_slam/ORB_SLAM2
./build.sh

echo "Launching Terminator..."

############################################################
# GAZEBO
############################################################

terminator --new-tab -x bash -c "
source ~/ssv2_ws/devel/setup.bash
echo '=== GAZEBO ==='
roslaunch aws_robomaker_small_house_world small_house.launch
exec bash" &

sleep 15

############################################################
# SLAM + RVIZ
############################################################

terminator --new-tab -x bash -c "
source ~/ssv2_ws/devel/setup.bash
echo '=== SLAM + RVIZ ==='
roslaunch semantic_slam slam.launch fixed_frame:=map
exec bash" &

sleep 8

############################################################
# SEMANTIC MAPPING (ORB SLAM)
############################################################

terminator --new-tab -x bash -c "
source ~/ssv2_ws/devel/setup.bash
echo '=== SEMANTIC MAPPING ==='
roslaunch semantic_slam semantic_mapping.launch
exec bash" &

sleep 8

############################################################
# STATIC TRANSFORM
############################################################

terminator --new-tab -x bash -c "
source ~/ssv2_ws/devel/setup.bash
echo '=== MAP -> ODOM TF ==='
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
exec bash" &

sleep 3

############################################################
# MOVE BASE
############################################################

terminator --new-tab -x bash -c "
source ~/ssv2_ws/devel/setup.bash
echo '=== MOVE BASE ==='
roslaunch jackal_2dnav move_base.launch
exec bash" &

sleep 5

############################################################
# CONTROL MODE
############################################################

MODE="$1"

# If no parameter supplied, ask interactively
if [ -z "$MODE" ]; then
    echo ""
    echo "Choose control mode:"
    echo "1) teleop"
    echo "2) auton"
    echo ""

    read -p "Enter choice: " MODE
fi

############################################################
# TELEOP MODE
############################################################

if [ "$MODE" = "teleop" ] || [ "$MODE" = "1" ]; then

    echo "Launching keyboard teleop..."

    terminator --new-tab -x bash -ic "
    source ~/.bashrc
    source ~/ssv2_ws/devel/setup.bash
    echo '=== TELEOP ==='
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    exec bash" &

############################################################
# AUTON MODE
############################################################

elif [ "$MODE" = "auton" ] || [ "$MODE" = "2" ]; then

    echo ""
    echo "Autonomous navigation mode enabled."
    echo ""

    # Optional initial velocity pulse
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

    sleep 1

    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

    echo "Next steps:"
    echo "1. In RViz set Fixed Frame = map"
    echo "2. Click '2D Nav Goal'"
    echo "3. Select a reachable goal"
    echo ""

############################################################
# INVALID OPTION
############################################################

else
    echo "Invalid option."
    echo "Usage:"
    echo "  start_ssv2"
    echo "  start_ssv2 teleop"
    echo "  start_ssv2 auton"
fi
############################################################
# FINAL MESSAGE
############################################################

echo ""
echo "========================================="
echo "SSV2 simulation stack launched."
echo "========================================="
echo ""
echo "Useful commands:"
echo "  rostopic echo /cmd_vel"
echo "  rostopic echo /move_base/status"
echo "  rosrun tf tf_echo map base_link"
echo ""
