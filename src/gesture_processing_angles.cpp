#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "convolution.h"
#include <iostream>
#include <fstream>
#include "math.h"
#include <vector>

std::string command;
int identifier;
bool command_received = false;

std::vector <float> right_pose = {3.0, -1.25, 1000, 1000};
std::vector <float> left_pose = {1000, 1000, 0.15, 1.5};
std::vector <float> forward_pose = {4.3, -2.45, -1.25, 0.12};
std::vector <float> stop_pose = {4.3, -2.45, 1000, 1000};


//info about all needed keypoints
openpose_ros_msgs::PointWithProb Neck, RShoulder, RElbow, RWrist, LShoulder, LElbow, LWrist;

float RSAngle, LSAngle, REAngle, LEAngle;

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

    //calculating points for angle of right shoulder
    float Neck_y_for_R_shoulder = Neck.y - RShoulder.y;
    float Neck_x_for_R_shoulder = Neck.x - RShoulder.x;
    float R_Elbow_y_for_R_shoulder = RElbow.y - RShoulder.y;
    float R_Elbow_x_for_R_shoulder = RElbow.x - RShoulder.x;

    //Right shoulder angle 
    float RSAngle1 = atan2(Neck_y_for_R_shoulder, Neck_x_for_R_shoulder); 
    float RSAngle2 = atan2(R_Elbow_y_for_R_shoulder, R_Elbow_x_for_R_shoulder);
    if (RSAngle2 < 0)
    {
      RSAngle2 = 3.14159 + (RSAngle2 + 3.14159);
    }
    RSAngle = RSAngle1 + RSAngle2;
    ROS_INFO_STREAM("Right shoulder is ");
    ROS_INFO_STREAM(RSAngle);
    //Right shoulder angle done

    //calculating points for angle of right elbow
    float R_shoulder_y_for_R_elbow = RShoulder.y - RElbow.y;
    float R_shoulder_x_for_R_elbow = RShoulder.x - RElbow.x;
    float R_wrist_y_for_R_elbow = RWrist.y - RElbow.y;
    float R_wrist_x_for_R_elbow = RWrist.x - RElbow.x;

    //Right elbow angle
    float REAngle1 = atan2(R_shoulder_y_for_R_elbow, R_shoulder_x_for_R_elbow);
    float REAngle2 = atan2(R_wrist_y_for_R_elbow, R_wrist_x_for_R_elbow);
    REAngle = REAngle2 - REAngle1;
    ROS_INFO_STREAM("Right elbow is ");
    ROS_INFO_STREAM(REAngle);
    //Right elbow angle done

    //calculating points for angle of left shoulder
    float Neck_y_for_L_shoulder = LShoulder.y - Neck.y; //right now this and lower line have incorrect naming of variable
    float Neck_x_for_L_shoulder = LShoulder.x - Neck.x;
    float L_Elbow_y_for_L_shoulder = LElbow.y - LShoulder.y;
    float L_Elbow_x_for_L_shoulder = LElbow.x - LShoulder.x;

    //Left shoulder angle
    float LSAngle1 = atan2(Neck_y_for_L_shoulder, Neck_x_for_L_shoulder);
    float LSAngle2 = atan2(L_Elbow_y_for_L_shoulder, L_Elbow_x_for_L_shoulder);
    LSAngle = LSAngle1 + LSAngle2;
    ROS_INFO_STREAM("Left shoulder is ");
    ROS_INFO_STREAM(LSAngle);
    //Left shoulder angle done

    //calculating points for angle of left elbow
    float L_wrist_y_for_L_elbow = LWrist.y - LElbow.y;
    float L_wrist_x_for_L_elbow = LWrist.x - LElbow.x;

    //Left elbow angle
    float LEAngle1 = atan2(L_Elbow_y_for_L_shoulder, L_Elbow_x_for_L_shoulder);
    float LEAngle2 = atan2(L_wrist_y_for_L_elbow, L_wrist_x_for_L_elbow);
    LEAngle = LEAngle1 - LEAngle2;
    ROS_INFO_STREAM("Left elbow is ");
    ROS_INFO_STREAM(LEAngle);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gesture_processer");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::AsyncSpinner spinner(4);
	  spinner.start();
  return 0;
  while (ros::ok)
  {
//Need to be changed (wrong)
    float left_pose_s, right_pose_s, forward_pose_s, stop_pose_s;
    int None_values_cnt_right = 0;
    for (float point: right_pose)
    {
      if (point != 1000)
      {
        if (point < RSAngle)
        {
          right_pose_s += RSAngle - point; 
        }
        else
        {
          right_pose_s += point - RSAngle;
        }
      }
      else
      {
        None_values_cnt_right += 1;
      }
      
      }
    }
  }
}