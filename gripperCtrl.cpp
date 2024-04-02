//Translation of Fetch gripperctrl.py

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>


// Our Action interface type for moving Fetch's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_control_client;

float gripper_closed = 0.00;
float gripper_open = 0.05;

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "gripper_control");
  ros::NodeHandle nh;

  ROS_INFO("Waiting for gripper controller...");
  gripper_control_client gripper_client("/gripper_controller/gripper_action", true);
  gripper_client.waitForServer(); //will wait for infinite time
  ROS_INFO("...connected");

  // send a goal to the action
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.max_effort = 10.0;
  gripper_goal.command.position = gripper_closed;


  ROS_INFO("Setting positions closed...");
  // Sends the command to start the given trajectory 1s from now
  gripper_client.sendGoal(gripper_goal);

  gripper_client.waitForResult(ros::Duration(5.0));


  // send a goal to the action
  //control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.max_effort = 10.0;
  gripper_goal.command.position = gripper_open;


  ROS_INFO("Setting positions open...");
  // Sends the command to start the given trajectory 1s from now
  gripper_client.sendGoal(gripper_goal);

  gripper_client.waitForResult(ros::Duration(5.0));

  ROS_INFO("...done");
  

  return 0;
}