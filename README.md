# RoboND-HomeServiceRobot-Project  
This is the final project of the Robotics Software Nanodegree Program by Udacity.

## Directory structure  
```
    ├──                                # Official ROS packages  
    |  
    ├── slam_gmapping                  # gmapping_demo.launch file                     
    │   ├── gmapping  
    │   ├── ...  
    ├── turtlebot                      # keyboard_teleop.launch file  
    │   ├── turtlebot_teleop  
    │   ├── ...  
    ├── turtlebot_interactions         # view_navigation.launch file        
    │   ├── turtlebot_rviz_launchers  
    │   ├── ...  
    ├── turtlebot_simulator            # turtlebot_world.launch file   
    │   ├── turtlebot_gazebo  
    │   ├── ...  
    │  
    ├──                                # Your packages and direcotries  
    |
    ├── map                            # map files  
    │   ├── ...  
    ├── scripts                        # shell scripts files  
    │   ├── ...  
    ├──rvizConfig                      # rviz configuration files  
    │   ├── ...  
    ├──pick_objects                    # pick_objects C++ node  
    │   ├── src/pick_objects.cpp  
    │   ├── ...  
    ├──add_markers                     # add_marker C++ node  
    │   ├── src/add_markers.cpp  
    │   ├── ...  
    └──  
 ```
Home Service Robot 
This is the final project of Robotics Software Engineer Nanodegree Program by Udacity.

Project Description
The project attempts to combine SLAM and Navigation into a robot so that it can autonomously transport an object in a Gazebo environment. By running home_service.sh file, the robot performs the following tasks:
1.	Navigating to a object at the pickup zone
2.	Picking the object.
3.	Carrying the object to the drop off zone.
4.	Dropping off the object to the zone.






Simulation Set Up
This repository consists of the following packages and directories:
1. Official ROS packages
	gmapping
	turtlebot_teleop		turtlebot_rviz_launchers
	turtlebot_gazebo 
2. My packages and directories
	map: Stores a gazebo world file and a map generated from SLAM.
	scripts: Stores shell script (.sh) files.
	rvizConfig: Stores customized Rviz configuration files.
	pick_objects: a node that commands the robot to drive to the pickup and drop off zones.
	add_markers: a node that model the object with a marker in rviz.

Preparation for the home service simulation 
1. SLAM Testing
Run test_slam.sh to manually performs SLAM and create a map of the Gazebo environment.
2. Localization & Navigation Testing
Run test_navigation.sh to make sure the robot is capable of estimating a robot's position relative to the map with AMCL and reaching multiple goals by manually commanding it with 2D Nav Goal in Rviz.
     
Behind the scenes of home_service.sh
The home_service.sh file runs the following 2 nodes:
	pick_objects: communicates with the ROS navigation stack, and autonomously sends pickup and drop off goals for the robot.
	add_markers: publishes/hides a marker (a virtual object) at the pickup and drop off zone, and subscribes to robot's odometry and keeps track of the robot pose.
So in actually, the performed tasks are as follows: 
1.	add_markers node publishes a marker (a virtual object) at the pickup zone
2.	pick_objects node commands the robot to drive the pickup zone
3.	add_markers node hides the marker once the robot reaches the pickup zone. 
4.	add_markers node wait 5 seconds to simulate a pickup.
  5.  add_markers node shows the marker at the drop off zone once the robot reaches it

