// This program publishes randomly-generated velocity
// messages for turtlesim.
// From: https://www.cse.sc.edu/~jokane/agitr/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <sensor_msgs/LaserScan.h>

void laserScanReceived(const sensor_msgs::LaserScan& msg) {
    int i;
    for(i=0;i < msg.ranges.size();i++){
    }
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "/cmd_vel", 10);

  // sub
    ros::Subscriber sub = nh.subscribe("/base_scan", 1000,
                                       &laserScanReceived);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);
  while(ros::ok()) {
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::Twist msg;
    /*
    msg.linear.x = double(rand())/double(RAND_MAX);
    msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
     */

    //msg.linear.x = (front/maxRange)*maxSpeed;
    // !isinf to tell if its (not) infite
    //msg.linear.x = (front/20)*maxSpeed;
    //msg.angular.z =

    // Publish the message.
    pub.publish(msg);
    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

    //ros::spinOnce();
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
