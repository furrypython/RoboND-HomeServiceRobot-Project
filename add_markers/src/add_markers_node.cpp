#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class addMarkers{
public:
  addMarkers(){
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    odom_pub = n.subscribe("odom", 10, addmarkers::odomCallback, this);
  }
  
  void setMarker(visualization_msgs::Marker& marker){
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
  
  void setPose(visualization_msgs::Marker& marker, double posX,double posY, double oriW) {
    // Set the epose of the marker
    marker.pose.position.x = poseX;
    marker.pose.position.y = poseY;
    marker.pose.orientation.w = oriW;
  }
  
  void odomCallback(const nav_msgs::Odometry& odom){
    double robotPoseX = odom.pose.position.x;
    double robotPoseY = odom.pose.position.y;
    double diff = 0.00001;
  
    double dpx = abs(robotPoseX - pickup.x);
    double dpy = abs(robotPoseY - pickup.y);
    if(!reachPickup && dpx < diff && dpy < diff){
      ROS_INFO("Reached the pickup zone");
      reachPickup = true;
    }
  
    double ddx = abs(robotPoseX - dropoff.x);
    double ddy = abs(robotPoseY - dropoff.y);
    if(!reachDropoff && ddx < diff && ddy < diff){
      ROS_INFO("Reached the dropoff zone");
      reachDropoff = true;
    }  
  }
  
private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_pub;
  bool reachPickup = false;
  bool reachDropoff = false;
};

int main( int argc, char** argv ){
  ros::init(argc, argv, "add_markers");
  ros::Rate r(1);

  if (!ros::ok()) {
    return 0;
  }

  visualization_msgs::Marker marker;

  addMarkers::setMarker(marker);
  addMarkers::waitSub(marker_pub);

  // -----------------------------------------------------------
  // Publish the marker at the pickup zone
  addMarkers::setPose(marker, -0.488, -4.103, 1.0);
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);

  // Check if reach the pickup zone
  while(!reachPickup){
      ros::spin();
      sleep(1);
  }
  
  // Picking...
  // Pause 5 seconds
  ros::Duration(5.0).sleep();
  
  // ----------------------------------------------------------
  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker picked");
  
  // Check if reach the dropoff zone
  while(!reachDropoff){
      ros::spin();
      sleep(1);
  }
  
  // Dropping... 
  // Pause 5 seconds
  ros::Duration(5.0).sleep();
  
  // ----------------------------------------------------------
  // Publish the marker at the dropoff zone
  addMarkers::setPose(marker, 5.281, -2.463, 1.0);
  addMarkers::marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);

  ROS_INFO("Marker reached the dropoff zone");

  return 0;
}
