#!/bin/bash

set -e

source ~/.bashrc
source ~/ssv2_ws/devel/setup.bash

echo "Building ORB_SLAM2..."
cd ~/ssv2_ws/src/semantic_slam/ORB_SLAM2
./build.sh

WS_SETUP="source ~/.bashrc; source ~/ssv2_ws/devel/setup.bash"

echo "Launching Terminator..."

############################################################
# CONTROL
############################################################

terminator --new-tab -x bash -ic "
$WS_SETUP
echo '=== CONTROL ==='
roslaunch jackal_control control.launch
exec bash
" &

sleep 10

############################################################
# SLAM (MUST come before NAV)
############################################################

terminator --new-tab -x bash -ic "
$WS_SETUP
echo '=== SLAM ==='
roslaunch semantic_slam slam.launch fixed_frame:=map
exec bash
" &

sleep 10

############################################################
# NAVIGATION
############################################################

terminator --new-tab -x bash -ic "
$WS_SETUP
echo '=== MOVE BASE ==='
roslaunch jackal_2dnav move_base.launch
exec bash
" &

sleep 5

############################################################
# RVIZ / MAPPING LAST
############################################################

terminator --new-tab -x bash -ic "
$WS_SETUP
echo '=== RVIZ ==='
roslaunch semantic_slam semantic_mapping.launch
exec bash
" &
