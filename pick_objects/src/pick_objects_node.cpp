#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal setGoal(double poseX, double poseY, double poseW){
  move_base_msgs::MoveBaseGoal goal;  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define the 1st position and orientation for the robot to reach
  goal.target_pose.pose.position.x = poseX;
  goal.target_pose.pose.position.y = poseY;
  goal.target_pose.pose.orientation.w = poseW;

  return goal;
}

void sendAndCheck(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal, std::string target){
  ROS_INFO_STREAM("Sending the " << target << " goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("Hooray, reached the " << target << " zone!");
  } else {
    ROS_INFO_STREAM("Failed to reach the " << target << " zone.");
  }
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait infinite time for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  };
  ROS_INFO("Action server started");

  // Define the 1st position and orientation for the robot to reach
  move_base_msgs::MoveBaseGoal pickGoal = setGoal(-0.488, -4.103, 1.0);

  // Send the goal position and orientation for the robot to reach
  sendAndCheck(ac, pickGoal, "pickup");
  
  // Wait 5 seconds
  ros::Duration(5.0).sleep();

  // Define the 2nd position and orientation for the robot to reach
  move_base_msgs::MoveBaseGoal dropGoal = setGoal(5.281, -2.463, -1.0);

  // Send the goal position and orientation for the robot to reach
  sendAndCheck(ac, dropGoal, "drop off");

  return 0;
}
