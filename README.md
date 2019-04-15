# Home Service Robot Project  
This is the final project of Robotics Software Nanodegree Program by Udacity.

## Project Description  
The project attempts to combine SLAM and Navigation into a robot so that it can autonomously transport
an object in a Gazebo environment. By running `home_service.sh` file, the robot performs the
following tasks:  
1. Navigating to a object at the pickup zone.  
2. Picking the object.  
3. Carrying the object to the drop off zone.  
4. Dropping off the object to the zone.  

## Home Service Simulation
``` 
$ cd <YOUR CATKIN WORKSPACE>/src
$ git -clone https://github.com/reiomori/RoboND-HomeServiceRobot-Project
$ cd ..
$ catkin_make
$ cd src/RoboND-HomeServiceRobot-Project/scripts
$ chmod +x home_service.sh
$ ./home_service.sh
```  


