// This program publishes randomly-generated velocity
// messages for turtlesim.
// From: https://www.cse.sc.edu/~jokane/agitr/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>// For geometry_msgs::Twist
#include <opencv_apps/MomentArrayStamped.h>
#include <stdlib.h> // For rand() and RAND_MAX
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <exception>
#include <string>
#include <tf2/LinearMath/Quaternion.h>

float xCo;
float yCo;
float xAr;
static const std::string OPENCV_WINDOW = "Image window";
//sensors
float minfront;
float minright;
float minleft;
float maxfront;
float maxright;
float maxleft;
float colourdetector;
bool colour = false;
int state = 0;
float tiltval;
float y;
int BlocksReturned = 0;

void poseMessageReceived(const opencv_apps::MomentArrayStamped& msg) {
    //colourdetector = msg.moments.size();
    if (msg.moments.size() > 0){
        xCo = msg.moments[0].center.x;
        yCo = msg.moments[0].center.y;
        xAr = msg.moments[0].area;
        colour = true;
    }
}

void laserScanReceived(const sensor_msgs::LaserScan& msg) {
    minfront = 25.0;
    maxfront = 0;
    for(int i = 310; i<=350;i++){
        if (minfront > msg.ranges[i]){
            minfront = msg.ranges[i];
        }
        if(maxfront < msg.ranges[i]){// just collecting the maxes, it's not used but it thought it would be useful
            maxfront = msg.ranges[i];
        }
    }
    minright = 25.0;
    maxright =0;
    for(int i = 510; i<=550;i++){
        if (minright > msg.ranges[i]){
            minright= msg.ranges[i];
        }
        if(maxright < msg.ranges[i]){
            maxright= msg.ranges[i];
        }
    }

    minleft = 25.0;
    maxleft = 0.0;
    for(int i = 120; i<=160;i++){
        if (minleft > msg.ranges[i]){
            minleft= msg.ranges[i];
        }
        if(maxleft < msg.ranges[i]){
            maxleft= msg.ranges[i];
        }
    }
}

// code for the head
void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    //This function uses x and y coordinates from the contour moments
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    float depth = cv_ptr->image.at<float>(yCo,xCo);
    // for verification purposes:
    // draw circle of radius 10 around the xy point
    cv::circle(cv_ptr->image, cv::Point(xCo, yCo), 10, CV_RGB(255,0,0));
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    //needs work
    //waitForMessage (const std::string sensor_msgs/CameraInfo&, NodeHandle &nh, ros::Duration timeout);
    static tf2_ros::TransformBroadcaster br;
    float f = 554.254691191187;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "head_camera_depth_frame";
    transformStamped.child_frame_id = "target_object";




    transformStamped.transform.translation.x = depth;
    //height 480
    //weight 640
    transformStamped.transform.translation.y = -(xCo-(640/2)/f)*depth;
    transformStamped.transform.translation.z = -(yCo-(480/2)/f)*depth;
    tf2::Quaternion q;
    //setRPY(roll, pitch, yaw) calculates quaternion rotation for transform
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
    tiltval = depth;
}

// Our Action interface type for moving Fetch's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;

std::string head_joint_names[2] = {"head_pan_joint", "head_tilt_joint"};

float head_joint_positions[3][2] = {//these are the head positions from the prac there not used i this program but i did use them at some point
        {-1.57, -0.76} ,
        {1.57, 1.45} ,
        {0.0, 0.0}
};

/* this commented out code was going to be a function that resets the head
void HeadReset(control_msgs::FollowJointTrajectoryGoal head_goal){
    //reset head
    head_goal.trajectory.joint_names.push_back(head_joint_names[0]);
    head_goal.trajectory.joint_names.push_back(head_joint_names[1]);
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0].positions.resize(1);
    head_goal.trajectory.points[0].positions[1] = 0;
    head_goal.trajectory.points[0].positions[0] = 0;
    head_goal.trajectory.points[0].velocities.resize(1);
    head_goal.trajectory.points[0].velocities[0] = 1;
    head_goal.trajectory.points[0].velocities[1] = 1;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(5.0);
    head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    head_client.sendGoal(head_goal);
    // Wait for trajectory execution
    head_client.waitForResult(ros::Duration(2.0));
}*/

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;
  ros::NodeHandle tb;

  // Create a publisher object.
  ros::Publisher pub = tb.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  // sub
  ros::Subscriber base = tb.subscribe("/base_scan", 1000,&laserScanReceived);

  ros::Subscriber sub1 = tb.subscribe("/contour_moments_red/moments", 1000, &poseMessageReceived);//used to be nh.
  //ros::Subscriber sub2 = tb.subscribe("/contour_moments_green/moments", 1000, &poseMessageReceived); //this is for the green object

  head_control_client head_client("/head_controller/follow_joint_trajectory", true);
  head_client.waitForServer();

  control_msgs::FollowJointTrajectoryGoal head_goal;
  //reset head
  head_goal.trajectory.joint_names.push_back(head_joint_names[0]);
  head_goal.trajectory.joint_names.push_back(head_joint_names[1]);
  head_goal.trajectory.points.resize(1);
  head_goal.trajectory.points[0].positions.resize(1);

  head_goal.trajectory.points[0].positions[1] = 0;
  head_goal.trajectory.points[0].positions[0] = 0;
  head_goal.trajectory.points[0].velocities.resize(1);
  head_goal.trajectory.points[0].velocities[0] = 1;
  head_goal.trajectory.points[0].velocities[1] = 1;
  head_goal.trajectory.points[0].time_from_start = ros::Duration(5.0);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  head_client.sendGoal(head_goal);
  // Wait for trajectory execution
  head_client.waitForResult(ros::Duration(2.0));


  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(10);
  while(ros::ok() && BlocksReturned < 2) {
    // Create and fill in the message.  The other four
    geometry_msgs::Twist msg;

    float x;
    switch(state){
        case 0:
            msg.linear.x = 0.08*minfront;
            msg.angular.z = 0.1*(minleft-minright);
            if(colour){
                state = 1;
                base.shutdown();
            }
            break;
        case 1:
            //rest head
            //ros::Subscriber sub1 = tb.subscribe("/contour_moments_red/moments", 1000, &poseMessageReceived); //resubscribing
            x = 320-xCo;
            y = 240-yCo;
            if(xAr < 8300){
                msg.linear.x = 0.35;
                msg.angular.z = x*0.005;
            }
            else if(xAr > 8300){
                msg.linear.x = 0;
                msg.angular.z = 0;
                state = 2;
                //sub1..shutdown();
            }
            if(y < -220.0){
                head_goal.trajectory.joint_names.push_back(head_joint_names[0]);
                head_goal.trajectory.joint_names.push_back(head_joint_names[1]);
                head_goal.trajectory.points.resize(1);
                head_goal.trajectory.points[0].positions.resize(1);

                head_goal.trajectory.points[0].positions[1] = 1.1;
                head_goal.trajectory.points[0].positions[0] = 0;
                head_goal.trajectory.points[0].velocities.resize(2);
                head_goal.trajectory.points[0].velocities[0] = 1;
                head_goal.trajectory.points[0].velocities[1] = 1;
                head_goal.trajectory.points[0].time_from_start = ros::Duration(5.0);
                head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
                head_client.sendGoal(head_goal);
                // Wait for trajectory execution
                head_client.waitForResult(ros::Duration(2.0));
            }
            break;
        case 2:
            msg.linear.x = 0;//just used to stop the robot and indicate its switched states
            msg.angular.z = 0;
            //arm code goes here
            //once the arm has picked up the red block it'll turn 180
            //state = 3;

            break;
        case 3:
            //take red block to green object
            //reset head
            //ros::Subscriber sub2 = tb.subscribe("/contour_moments_green/moments", 1000, &poseMessageReceived);
            x = 320-xCo;
            y = 240-yCo;
            if(xAr < 8300){
                msg.linear.x = 0.35;
                msg.angular.z = x*0.005;
            }
            else if(xAr > 8300){
                msg.linear.x = 0;
                msg.angular.z = 0;
                state = 2;
                //sub2..shutdown();
                //once the arm lets go of the redbox turn 180
                //BlocksReturned + 1;
                //state = 1
            }
            //head movement will need to go here
            break;
    }

    // Publish the message.
    pub.publish(msg);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending velocity command:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z
      << " state=" << state
      << " xAR=" << xAr
      << " y" << y);

      // Subscribe to input image topic using image transport
      image_transport::ImageTransport it(nh);
      image_transport::Subscriber depth_sub =
              it.subscribe("/head_camera/depth_registered/image_raw", 1, imageCb);
      cv::namedWindow(OPENCV_WINDOW);
      //ros::Subscriber sub = nh.subscribe("/head_camera/depth_registered/image_raw", 1, &imageCb);
      //ros::init(argc, argv, "tf2_broadcaster");

    // Wait until it's time for another iteration.
    ros::spinOnce();
    rate.sleep();
  }
}
