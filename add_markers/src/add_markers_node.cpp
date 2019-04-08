#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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
pose dropoff = {5.281, -2.463, 0.0, 0.0, 0.0, 1.0};

void setMarker(visualization_msgs::Marker& marker)
{
  // Set the initial shape type to be a cube
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

  // Set the epose of the marker
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.pose.position.z = goal.z;
  marker.pose.orientation.x = goal.ox;
  marker.pose.orientation.y = goal.oy;
  marker.pose.orientation.z = goal.oz;
  marker.pose.orientation.w = goal.ow;
}

void odomCallback(){
  
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_pub = n.subscribe("odom", 10, odomCallback);

  if (!ros::ok()) {
    return 0;
  }

  visualization_msgs::Marker marker;

  setMarker(marker);
  waitSub(marker_pub);

  // Publish the marker at the pickup zone
  setPose(marker, pickup);
  marker_pub.publish(marker);

  // Pause 5 seconds
  ros::Duration(5.0).sleep();

  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker picked");

  // Pause 5 seconds
  ros::Duration(5.0).sleep();

  // Publish the marker at the dropoff zone
  setPose(marker, dropoff);

  // Publish the marker
  marker_pub.publish(marker);

  ROS_INFO("Marker reached the dropoff zone");

  return 0;
}
