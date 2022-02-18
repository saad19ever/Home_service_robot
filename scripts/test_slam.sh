#!/bin/sh
xterm  -e  " roslaunch turtlebot3_gazebo turtlebot3_myworld.launch "  &
sleep 5
xterm  -e  "  roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot3_slam turtlebot3_slam.launch" &

