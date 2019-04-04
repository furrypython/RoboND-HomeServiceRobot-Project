#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find map)/building.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/building.yaml

xterm  -e  "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch pick_objects pick_objects_node"
