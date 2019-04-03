#!/bin/sh
xterm  -e  "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/building.world" &
sleep 5
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/building.yaml" &
sleep 5
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch"
