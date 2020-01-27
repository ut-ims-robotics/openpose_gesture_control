#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "convolution.h"
#include <iostream>
#include <fstream>

std::string command;
int identifier;
bool command_received = false;

//info about all needed keypoints
openpose_ros_msgs::PointWithProb Neck, RShoulder, RElbow, RWrist, LShoulder, LElbow, LWrist;

float RSAngle, LSAngle, RWAngle, LWAngle;

geometry_msgs::Twist msg;
float values[10];

void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& pose_msg)
{
  if (pose_msg.num_humans == 0)
	{
		command_received = false;
	}
  else
  {
    command_received = true;

    Neck = pose_msg.human_list[0].body_key_points_with_prob[1];
    RShoulder = pose_msg.human_list[0].body_key_points_with_prob[2];
    RElbow = pose_msg.human_list[0].body_key_points_with_prob[3];
    RWrist = pose_msg.human_list[0].body_key_points_with_prob[4];
    LShoulder = pose_msg.human_list[0].body_key_points_with_prob[5];
    LElbow = pose_msg.human_list[0].body_key_points_with_prob[6];
    LWrist = pose_msg.human_list[0].body_key_points_with_prob[7];

    //calculating angle of right shoulder
    float Neck_y_for_R_sholder = Neck.y - RShoulder.y;
    float Neck_x_for_R_sholder = Neck.x - RShoulder.x;
    float Wrist_y_for_R_sholder = RWrist.y - RShoulder.y;
    float Wrist_x_for_R_sholder = RWrist.x - RShoulder.x;

    float RSAngle1 = atan2(Neck_y_for_R_sholder, Neck_x_for_R_sholder);
    float RSAngle2 = atan2(Wrist_y_for_R_sholder, Wrist_x_for_R_sholder);
    RSAngle = RSAngle1 + RSAngle2;
    ROS_INFO_STREAM("angle of right shoulder is: " << RSAngle);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
  ros::AsyncSpinner spinner(4);
	  spinner.start();
  return 0;
}