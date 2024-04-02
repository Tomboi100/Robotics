// This program subscribes to turtle1/pose and shows its
// messages on the screen.
// From: https://www.cse.sc.edu/~jokane/agitr/
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <iomanip> // for std::setprecision and std::fixed

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const sensor_msgs::LaserScan& msg) {
    if(!isinf(msg.ranges[26])){
        ROS_INFO_STREAM(msg.ranges[26]);
    }
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_pose");
  ros::NodeHandle nh;

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("/velodyne/scan", 1000,
    &poseMessageReceived);

  // Let ROS take over.
  ros::spin();
}

