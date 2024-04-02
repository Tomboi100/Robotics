//Translation by phs of simple_disco.py, original by Fetch Robotics

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


std::vector<std::string> object_ids = {"my_front_ground", "my_back_ground", "my_right_ground", "my_left_ground"};



// Entry point
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hi");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface move_group("arm_with_torso");


	ROS_INFO("Removing existing objects from planning scene");
	planning_scene_interface.removeCollisionObjects(object_ids);


	ROS_INFO("Adding ground to planning scene");

    // Adding/Removing Objects and Attaching/Detaching Objects
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Define a collision object ROS message.
	//moveit_msgs::CollisionObject collision_object;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(4);

	//collision_object.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	collision_objects[0].id = "my_front_ground";
	collision_objects[0].header.frame_id = "base_link";

	// Define a box to add to the world.
	collision_objects[0].primitives.resize(1);
	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[0].primitives[0].dimensions.resize(3);
	collision_objects[0].primitives[0].dimensions[0] = 2.0;
	collision_objects[0].primitives[0].dimensions[1] = 2.0;
	collision_objects[0].primitives[0].dimensions[2] = 2.0;

	// Define the pose of the object
	collision_objects[0].primitive_poses.resize(1);
	collision_objects[0].primitive_poses[0].position.x = 1.1;
	collision_objects[0].primitive_poses[0].position.y = 0.0;
	collision_objects[0].primitive_poses[0].position.z = -1.0;

	collision_objects[0].operation = collision_objects[0].ADD;



	// The id of the object is used to identify it.
	collision_objects[1].id = "my_back_ground";
	collision_objects[1].header.frame_id = "base_link";

	// Define a box to add to the world.
	collision_objects[1].primitives.resize(1);
	collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[1].primitives[0].dimensions.resize(3);
	collision_objects[1].primitives[0].dimensions[0] = 2.0;
	collision_objects[1].primitives[0].dimensions[1] = 2.0;
	collision_objects[1].primitives[0].dimensions[2] = 2.0;

	// Define the pose of the object
	collision_objects[1].primitive_poses.resize(1);
	collision_objects[1].primitive_poses[0].position.x = -1.2;
	collision_objects[1].primitive_poses[0].position.y = 0.0;
	collision_objects[1].primitive_poses[0].position.z = -1.0;

	collision_objects[1].operation = collision_objects[1].ADD;


	// The id of the object is used to identify it.
	collision_objects[2].id = "my_right_ground";
	collision_objects[2].header.frame_id = "base_link";

	// Define a box to add to the world.
	collision_objects[2].primitives.resize(1);
	collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[2].primitives[0].dimensions.resize(3);
	collision_objects[2].primitives[0].dimensions[0] = 2.0;
	collision_objects[2].primitives[0].dimensions[1] = 2.0;
	collision_objects[2].primitives[0].dimensions[2] = 2.0;

	// Define the pose of the object
	collision_objects[2].primitive_poses.resize(1);
	collision_objects[2].primitive_poses[0].position.x = 0.0;
	collision_objects[2].primitive_poses[0].position.y = 1.2;
	collision_objects[2].primitive_poses[0].position.z = -1.0;

	collision_objects[2].operation = collision_objects[2].ADD;



	// The id of the object is used to identify it.
	collision_objects[3].id = "my_left_ground";
	collision_objects[3].header.frame_id = "base_link";

	// Define a box to add to the world.
	collision_objects[3].primitives.resize(1);
	collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[3].primitives[0].dimensions.resize(3);
	collision_objects[3].primitives[0].dimensions[0] = 2.0;
	collision_objects[3].primitives[0].dimensions[1] = 2.0;
	collision_objects[3].primitives[0].dimensions[2] = 2.0;

	// Define the pose of the object
	collision_objects[3].primitive_poses.resize(1);
	collision_objects[3].primitive_poses[0].position.x = 0.0;
	collision_objects[3].primitive_poses[0].position.y = -1.2;
	collision_objects[3].primitive_poses[0].position.z = -1.0;

	collision_objects[3].operation = collision_objects[3].ADD;

	ROS_INFO("applying collision objects");
	planning_scene_interface.applyCollisionObjects(collision_objects);

	std::string gripper_frame = "wrist_roll_link";



	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the
	// end-effector.

	std::vector<geometry_msgs::Pose> gripper_poses;
	gripper_poses.resize(2);
	
	/*
	gripper_poses[0].position.x = 0.042;
	gripper_poses[0].position.y = 0.384;
	gripper_poses[0].position.z = 1.826;
	gripper_poses[0].orientation.x = 0.173;
	gripper_poses[0].orientation.y = -0.693;
	gripper_poses[0].orientation.z = -0.242;
	gripper_poses[0].orientation.w = 0.657;

	gripper_poses[1].position.x = 0.047;
	gripper_poses[1].position.y = 0.545;
	gripper_poses[1].position.z = 1.822;
	gripper_poses[1].orientation.x = -0.274;
	gripper_poses[1].orientation.y = -0.701;
	gripper_poses[1].orientation.z = 0.173;
	gripper_poses[1].orientation.w = 0.635;
	*/


	
	gripper_poses[0].orientation.w = 1.0;
	gripper_poses[0].position.x = 0.28;
	gripper_poses[0].position.y = -0.2;
	gripper_poses[0].position.z = 0.5;

	gripper_poses[1].orientation.w = 1.0;
	gripper_poses[1].position.x = 0.55;
	gripper_poses[1].position.y = -0.05;
	gripper_poses[1].position.z = 0.8;
	
	
	/*
	gripper_poses[0].orientation.w = 1.0;
	gripper_poses[0].position.x = 0.156;
	gripper_poses[0].position.y = 0.484;
	gripper_poses[0].position.z = 0.614;

	gripper_poses[1].orientation.w = 1.0;
	gripper_poses[1].position.x = 0.335;
	gripper_poses[1].position.y = 0.484;
	gripper_poses[1].position.z = 0.461;
	*/


	while(ros::ok())
	{
		for(int i=0; i<gripper_poses.size(); i++)
		{
			move_group.setPoseTarget(gripper_poses[i], gripper_frame);
			move_group.move();
		}
	}

	

	// Now, we call the planner to compute the plan and visualize it.
	// Note that we are just planning, not asking move_group
	// // to actually move the robot.
	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
}