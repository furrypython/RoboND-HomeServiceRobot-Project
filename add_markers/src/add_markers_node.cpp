#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <cmath> // to use abs()

struct pose{
  float x;
  float y;
  float z;
  float ox;
  float oy;
  float oz;
  float ow;
};

pose pickup = {-0.488, -4.103, 0.0, 0.0, 0.0, 1.0};
pose dropoff = {5.281, -2.463, 0.0, 0.0, 0.0, -1.0};

bool reachPickup = false;
bool reachDropoff = false;

void setMarker(visualization_msgs::Marker& marker)
{
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the marker type.
  marker.type = shape;

  // Set the frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  marker.ns = "add_markers_ns";
  marker.id = 0;

  // Set the scale of the marker
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color
  marker.color.r = 0.913f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
}

void waitSub(ros::Publisher& marker_pub){
  // Sleep until a subscriber is ready
  while (marker_pub.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
}

void setPose(visualization_msgs::Marker& marker, pose& goal){
  // Set the marker action. 
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.pose.position.z = goal.z;
  marker.pose.orientation.x = goal.ox;
  marker.pose.orientation.y = goal.oy;
  marker.pose.orientation.z = goal.oz;
  marker.pose.orientation.w = goal.ow;
}

void odomCallback(const nav_msgs::Odometry &odom){
  double robotPosX = odom.pose.pose.position.x;
  double robotPosY = odom.pose.pose.position.y;
  double diff = 0.00001;
  double dpx = abs(robotPosX - pickup.x);
  double dpy = abs(robotPosY - pickup.y);
  double ddx = abs(robotPosX - dropoff.x);
  double ddy = abs(robotPosY - dropoff.y);  

  if(!reachPickup && dpx < diff && dpy < diff){
      ROS_INFO("Robot reached the pickup zone");
      reachPickup = true;
  }

  if(!reachDropoff && ddx < diff && ddy < diff){
      ROS_INFO("Robot reached the dropoff zone");
      reachDropoff = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

  if (!ros::ok())
  {
    return 0;
  }

  visualization_msgs::Marker marker;

  setMarker(marker);
  waitSub(marker_pub);

  // Publish the marker at the pickup zone
  ROS_INFO("Marker at the pickup zone");
  setPose(marker, pickup);
  marker_pub.publish(marker);

  while(!reachPickup){
    ros::spinOnce();
    sleep(1);
  }

  // Pause 5 seconds
  ros::Duration(5.0).sleep();

  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker picked");

  while(!reachDropoff){
    ros::spinOnce();
    sleep(1);
  }

  // Pause 5 seconds
  ros::Duration(5.0).sleep();

  // Publish the marker at the dropoff zone
  setPose(marker, dropoff);
  marker_pub.publish(marker);
  ROS_INFO("Marker at the dropoff zone.");

  return 0;
}
