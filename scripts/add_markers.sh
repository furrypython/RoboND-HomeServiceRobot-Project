#!/bin/sh

xterm  -e  "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/roomU.world" &
sleep 15
xterm  -e  "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/roomU.yaml" &
sleep 15
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 15
xterm -e "cd $(pwd)/../..; source devel/setup.bash; rosrun add_markers add_markers_node" &
