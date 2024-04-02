//Translation of Fetch headctrl.py
//Based on Tiago's arm controller

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


// Our Action interface type for moving Fetch's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;



std::string head_joint_names[2] = {"head_pan_joint", "head_tilt_joint"};

float head_joint_positions[3][2] = {  
   {-1.57, -0.76} ,
   {1.57, 1.45} ,
   {0.0, 0.0}
};

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "head_control");
  ros::NodeHandle nh;

  ROS_INFO("Waiting for head controller...");
  head_control_client head_client("/head_controller/follow_joint_trajectory", true);
  head_client.waitForServer(); //will wait for infinite time
  ROS_INFO("...connected");

  // send a goal to the action


  for(int i=0; i<3; i++) //Loop through the head positions array
  {
    if(!(ros::ok()))
      break;

    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.trajectory.joint_names.push_back(head_joint_names[0]);
    head_goal.trajectory.joint_names.push_back(head_joint_names[1]);

    //positions
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0].positions.resize(2);
    head_goal.trajectory.points[0].positions[0] = head_joint_positions[i][0];
    head_goal.trajectory.points[0].positions[1] = head_joint_positions[i][1];

    // Velocities
    head_goal.trajectory.points[0].velocities.resize(2);
    for (int j = 0; j < 2; ++j)
    {
      head_goal.trajectory.points[0].velocities[j] = 0.0;
    }

    // To be reached 4 seconds after starting along the trajectory
    head_goal.trajectory.points[0].time_from_start = ros::Duration(5.0);

    ROS_INFO("Setting positions...");
    // Sends the command to start the given trajectory 1s from now
    head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    head_client.sendGoal(head_goal);

    // Wait for trajectory execution
    head_client.waitForResult(ros::Duration(5.0));
    // while(!(head_client.getState().isDone()) && ros::ok())
    // {
    //   ros::Duration(1).sleep(); // sleep for four seconds
    // }
    ROS_INFO("...done");
  }

  return 0;
}