#!/bin/sh
xterm  -e  " roslaunch turtlebot3_gazebo turtlebot3_myworld.launch " &
sleep 10
xterm  -e  " roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/map/map.yaml" & 
sleep 10
xterm  -e  "   rosparam load ~/catkin_ws/src/config/add_marker_config.yaml ; 
           rosrun add_markers add_markers" & 
sleep 5
xterm  -e  " rosrun pick_objects pick_objects" 
