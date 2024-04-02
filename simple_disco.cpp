//Translation by phs of simple_disco.py, original by Fetch Robotics

#include <string>

#include <ros/ros.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/MoveItErrorCodes.h>


// TF joint names
std::vector<std::string> joint_names = {"torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"};

// Lists of joint angles in the same order as in joint_names
std::vector<std::vector<double>> disco_poses = {{0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0},
                   {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                   {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                   {0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0},
                   {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
                   {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
                   {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0}};

std::vector<std::string> object_ids = {"my_front_ground", "my_back_ground", "my_right_ground", "my_left_ground"};


// Entry point
int main(int argc, char** argv)
{
	ros::init(argc, argv, "panda_arm_pick_place");
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


	// Now, let's add the collision object into the world
	ROS_INFO_NAMED("tutorial", "Add an object into the world");
	//planning_scene_interface.addCollisionObjects(collision_objects);

	
	

	for(int i=0; i<disco_poses.size(); i++)
	{
		if(!(ros::ok()))
      		break;
		move_group.setJointValueTarget(disco_poses.at(i));
		move_group.move(); //plan and execute in one step

	}
}