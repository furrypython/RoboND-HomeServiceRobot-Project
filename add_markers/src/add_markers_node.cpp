#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class addMarkers {
public:
    addMarkers() {
        //Topic to publish
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        // Subscribe to odom topic
        odom_pub = n.subscribe("odom", 10, addMarkers::odomCallback, this);
    }
  
    void setPose(double posX,double posY){
        // Set the pose of the marker
        marker.pose.position.x = posX;
        marker.pose.position.y = posY;
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    }
  
    void publishMarker(){
        // Sleep till a subscriber is ready and then publish the marker.
        while (marker_pub.getNumSubscribers() < 1){
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
    }
  
    void setMarker(){
        // Set the initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;

        // Set the marker type.
        marker.type = shape;
    
        // Set the marker action. 
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the frame ID and timestamp
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker
        marker.ns = "add_markers_ns";
        marker.id = 0;
    
        // Initially show the marker at the pickup zone
        setPose(pickup[0], pickup[1]);

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
        publishMarker();
    }
  
    void odomCallback(const nav_msgs::Odometry& odom){
        bool reachPickup = false;
        bool reachDropoff = false;
        double robotPosX = odom.pose.position.x;
        double robotPosY = odom.pose.position.y;
        double diff = 0.00001;  
        double dpx = abs(robotPosX - pickup[0]);
        double dpy = abs(robotPosY - pickup[1]);
        double ddx = abs(robotPosX - dropoff[0]);
        double ddy = abs(robotPosY - dropoff[1]);
        
        if(!reachPickup && dpx < diff && dpy < diff){
            ROS_INFO("Reached the pickup zone");
            reachPickup = true;
        }
  
        if(!reachDropoff && ddx < diff && ddy < diff){
            ROS_INFO("Reached the dropoff zone");
            reachDropoff = true;
        }
        
        // Check if reach the pickup zone
        while(!reachPickup){
            ros::spin();
            sleep(1);
        }
        
    }
  
private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_pub;
  visualization_msgs::Marker marker;
  double pickup[2] = {-0.488, -4.103};
  double dropoff[2] = {5.281, -2.463};
};

int main(int argc, char** argv ){
  // Initialize the add_maekers node
  ros::init(argc, argv, "add_markers");
  ros::Rate r(1);

  // Initially show the marker at the pickup zone
  addMarkers::setMarker();

  
  marker_pub.publish(marker);

  
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
