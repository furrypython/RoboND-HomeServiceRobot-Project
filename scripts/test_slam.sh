
#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch " & 
sleep 5
xterm  -e  " rosrun rviz rviz" &
sleep 5
xterm  -e  " rosrun rviz rviz"  &
sleep 5
xterm  -e  " rosrun rviz rviz" 
