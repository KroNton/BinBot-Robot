#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch freenect_launch freenect.launch " & 
sleep 5
xterm  -e  " roslaunch binbot_bringup robot_model.launch " &
sleep 5
xterm  -e  " roslaunch differential_drive_arduino gmapping.launch " 