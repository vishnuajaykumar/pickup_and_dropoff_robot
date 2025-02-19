#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

double robot_x = 0.0;
double robot_y = 0.0;

// Odom callback
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_x = msg->pose.pose.position.x;
  robot_y = msg->pose.pose.position.y;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Publisher for markers and subscriber for odometry
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

  const double PICKUP_X = 2.595;
  const double PICKUP_Y = -1.450;
  const double DROPOFF_X = -2.94;
  const double DROPOFF_Y = -2.29;
  const double THRESHOLD = 0.3;

  while (ros::ok())
  {
    bool picked_up = false, dropped_off = false;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Waiting for a subscriber to the marker topic...");
      ros::Duration(0.5).sleep();
    }

    // marker at the pickup zone
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    ROS_INFO("Publishing marker at pickup zone...");
    marker_pub.publish(marker);

  
    while (ros::ok())
    {
      ros::spinOnce();  

      // Check if robot has reached the pickup zone (using Euclidean distance)
      if (!picked_up && std::hypot(robot_x - PICKUP_X, robot_y - PICKUP_Y) < THRESHOLD)
      {
        ROS_INFO("Pickup reached! Hiding marker...");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ros::Duration(5.0).sleep();
        picked_up = true;
      }

      // Check if robot has reached the drop-off zone (using Euclidean distance)
      if (picked_up && !dropped_off && std::hypot(robot_x - DROPOFF_X, robot_y - DROPOFF_Y) < THRESHOLD)
      {
        ROS_INFO("Drop-off reached! Showing marker...");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = DROPOFF_X;
        marker.pose.position.y = DROPOFF_Y;
        marker_pub.publish(marker);
        ros::Duration(5.0).sleep();
        dropped_off = true;
      }

      // Reset
      if (picked_up && dropped_off)
      {
        ROS_INFO("Resetting cycle...");
        ros::Duration(2.0).sleep();
        break;
      }

      r.sleep();
    }
  }

  return 0;
}
