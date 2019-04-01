
#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " rosrun gmapping gmapping_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch " & 
sleep 5
xterm  -e  " rosrun rviz rviz" 
