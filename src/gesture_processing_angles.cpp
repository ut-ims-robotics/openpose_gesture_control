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
#include "yaml-cpp/yaml.h"

std::string command;
int identifier;
bool command_received = false;

//for arm check later
bool RArm = false;
bool LArm = false;

//declaring vecors for body parts
//Right hand
std::vector <float> Neck_RShoulder = {0.0, 0.0};
std::vector <float> RElbow_RShoulder = {0.0, 0.0};
std::vector <float> RShoulder_RElbow = {0.0, 0.0};
std::vector <float> RWrist_RElbow = {0.0, 0.0};
// Left  hand
std::vector <float> Neck_LShoulder = {0.0, 0.0};
std::vector <float> LElbow_LShoulder = {0.0, 0.0};
std::vector <float> LShoulder_LElbow = {0.0, 0.0};
std::vector <float> LWrist_LElbow = {0.0, 0.0};

//Here all 4 angles are stored
std::vector <float> angles = {0.0, 0.0, 0.0, 0.0};
//Order: RShoulder, RElbow, LShoulder, LElbow

//for later use
std::vector <float> ref_angles_and_pose_name;

//For yaml parser
YAML::Node gestures;

std::vector <float> right_pose = {3.0, -1.25, 1000, 1000};
std::vector <float> left_pose = {1000, 1000, 0.15, 1.5};
std::vector <float> forward_pose = {4.3, -2.45, -1.25, 0.12};
std::vector <float> stop_pose = {4.3, -2.45, 1000, 1000};


//info about all needed keypoints
openpose_ros_msgs::PointWithProb Neck, RShoulder, RElbow, RWrist, LShoulder, LElbow, LWrist;

float RSAngle, LSAngle, REAngle, LEAngle;

float values[10];

float remap(float value, float start1, float end1, float start2, float end2)
{
  float outgoing = start2 + (end2 - start2) * ((value - start1) / (end1 - start1));
  return outgoing;
}

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

    //finding all necessary vectors for right arm
    if (RShoulder.x == 0.0 || RElbow.x == 0.0 || RWrist.x == 0.0)
    {
      RArm = false;
    }
    else
    {
      RArm = true;
      //0 - x    1 - y
      //vectors for shoulder angle
      Neck_RShoulder[0] = Neck.x - RShoulder.x;
      Neck_RShoulder[1] = Neck.y - RShoulder.y;
      RElbow_RShoulder[0] = RElbow.x - RShoulder.x;
      RElbow_RShoulder[1] = RElbow.y - RShoulder.y;
      //vectors for elbow angle
      RShoulder_RElbow[0] = RShoulder.x - RElbow.x;
      RShoulder_RElbow[1] = RShoulder.y - RElbow.y;
      RWrist_RElbow[0] = RWrist.x - RElbow.x;
      RWrist_RElbow[1] = RWrist.y - RElbow.y;
    }

    //finding all necessary vectors for left arm
    if (LShoulder.x == 0.0 || LElbow.x == 0.0 || LWrist.x == 0.0)
    {
      LArm = false;
    }
    else
    {
      LArm = true;
      //0 - x    1 - y
      //vectors for shoulder angle
      Neck_LShoulder[0] = Neck.x - LShoulder.x;
      Neck_LShoulder[1] = Neck.y - LShoulder.y;
      LShoulder_LElbow[0] = LElbow.x - LShoulder.x;
      LShoulder_LElbow[1] = LElbow.y - LShoulder.y;
      //vectors for elbow angle
      LElbow_LShoulder[0] = LShoulder.x - LElbow.x;
      LElbow_LShoulder[1] = LShoulder.y - LElbow.y;
      LWrist_LElbow[0] = LWrist.x - LElbow.x;
      LWrist_LElbow[1] = LWrist.y - LElbow.y;
    }

    //calculating right arm angles
    if (!RArm)
    {
      angles[0] = 0.0;
      angles[1] = 0.0;
    }
    else
    {
      float Neck_RShoulder_magnitude = sqrt(pow(Neck_RShoulder[0],2) + pow(Neck_RShoulder[1],2));
      float RElbow_RShoulder_magnitude = sqrt(pow(RElbow_RShoulder[0],2) + pow(RElbow_RShoulder[1],2));
      //angles[0] = acos((Neck_RShoulder[0]*RElbow_RShoulder[0] + Neck_RShoulder[1]*RElbow_RShoulder[1])/(Neck_RShoulder_magnitude*RElbow_RShoulder_magnitude));
      angles[0] = atan2(Neck_RShoulder[0]*RElbow_RShoulder[0] + Neck_RShoulder[1]*RElbow_RShoulder[1], Neck_RShoulder[0]*RElbow_RShoulder[1] - Neck_RShoulder[1]*RElbow_RShoulder[0]);

      float RShoulder_RElbow_magnitude = sqrt(pow(RShoulder_RElbow[0],2) + pow(RShoulder_RElbow[1],2));
      float RWrist_RElbow_magnitude = sqrt(pow(RWrist_RElbow[0],2) + pow(RWrist_RElbow[1],2));
      //angles[1] = acos((RShoulder_RElbow[0]*RWrist_RElbow[0] + RShoulder_RElbow[1]*RWrist_RElbow[1])/(RShoulder_RElbow_magnitude*RWrist_RElbow_magnitude));
      angles[1] = atan2(RShoulder_RElbow[0]*RWrist_RElbow[0] + RShoulder_RElbow[1]*RWrist_RElbow[1],RShoulder_RElbow[0]*RWrist_RElbow[1] - RShoulder_RElbow[1]*RWrist_RElbow[0]);
    }

    //calculating left arm angles
    if (!LArm)
    {
      angles[2] = 0.0;
      angles[3] = 0.0;
    }
    else
    {
      float Neck_LShoulder_magnitude = sqrt(pow(Neck_LShoulder[0],2) + pow(Neck_LShoulder[1],2));
      float LElbow_LShoulder_magnitude = sqrt(pow(LElbow_LShoulder[0],2) + pow(LElbow_LShoulder[1],2));
      //angles[2] = acos((Neck_LShoulder[0]*LShoulder_LElbow[0] + Neck_LShoulder[1]*LShoulder_LElbow[1])/(Neck_LShoulder_magnitude*LElbow_LShoulder_magnitude));
      angles[2] = atan2(Neck_LShoulder[0]*LElbow_LShoulder[0] + Neck_LShoulder[1]*LElbow_LShoulder[1], Neck_LShoulder[0]*LElbow_LShoulder[1] - Neck_LShoulder[1]*LElbow_LShoulder[0]);

      float LShoulder_LElbow_magnitude = sqrt(pow(LShoulder_LElbow[0],2) + pow(LShoulder_LElbow[1],2));
      float LWrist_LElbow_magnitude = sqrt(pow(LWrist_LElbow[0],2) + pow(LWrist_LElbow[1],2));
      //angles[3] = acos((LShoulder_LElbow[0]*LWrist_LElbow[0] + LShoulder_LElbow[1]*LWrist_LElbow[1])/(LShoulder_LElbow_magnitude*LWrist_LElbow_magnitude));
      angles[3] = atan2(LShoulder_LElbow[0]*LWrist_LElbow[0] + LShoulder_LElbow[1]*LWrist_LElbow[1],LShoulder_LElbow[0]*LWrist_LElbow[1] - LShoulder_LElbow[1]*LWrist_LElbow[0]); 
    }
    for (int i = 0; i < angles.size(); i++)
    {
      angles[i] = angles[i] * 180/M_PI;
      angles[i] = remap(angles[i], -180, 180, 0, 360);
    }
    std::cout << "RShoulder: " << angles[0] << std::endl << "RElbow: " << angles[1] << std::endl << "LShoulder: " << angles[2] << std::endl << "LElbow: " << angles[3] << std::endl;
    
    


    // prev version

    // //calculating points for angle of right shoulder
    // float Neck_y_for_R_shoulder = Neck.y - RShoulder.y;
    // float Neck_x_for_R_shoulder = Neck.x - RShoulder.x;
    // float R_Elbow_y_for_R_shoulder = RElbow.y - RShoulder.y;
    // float R_Elbow_x_for_R_shoulder = RElbow.x - RShoulder.x;

    // //Right shoulder angle 
    // float RSAngle1 = atan2(Neck_y_for_R_shoulder, Neck_x_for_R_shoulder); 
    // float RSAngle2 = atan2(R_Elbow_y_for_R_shoulder, R_Elbow_x_for_R_shoulder);
    // if (RSAngle2 < 0)
    // {
    //   RSAngle2 = 3.14159 + (RSAngle2 + 3.14159);
    // }
    // RSAngle = RSAngle1 + RSAngle2;
    // ROS_INFO_STREAM("Right shoulder is ");
    // ROS_INFO_STREAM(RSAngle);
    // //Right shoulder angle done

    // //calculating points for angle of right elbow
    // float R_shoulder_y_for_R_elbow = RShoulder.y - RElbow.y;
    // float R_shoulder_x_for_R_elbow = RShoulder.x - RElbow.x;
    // float R_wrist_y_for_R_elbow = RWrist.y - RElbow.y;
    // float R_wrist_x_for_R_elbow = RWrist.x - RElbow.x;

    // //Right elbow angle
    // float REAngle1 = atan2(R_shoulder_y_for_R_elbow, R_shoulder_x_for_R_elbow);
    // float REAngle2 = atan2(R_wrist_y_for_R_elbow, R_wrist_x_for_R_elbow);
    // REAngle = REAngle2 - REAngle1;
    // ROS_INFO_STREAM("Right elbow is ");
    // ROS_INFO_STREAM(REAngle);
    // //Right elbow angle done

    // //calculating points for angle of left shoulder
    // float Neck_y_for_L_shoulder = LShoulder.y - Neck.y; //right now this and lower line have incorrect naming of variable
    // float Neck_x_for_L_shoulder = LShoulder.x - Neck.x;
    // float L_Elbow_y_for_L_shoulder = LElbow.y - LShoulder.y;
    // float L_Elbow_x_for_L_shoulder = LElbow.x - LShoulder.x;

    // //Left shoulder angle
    // float LSAngle1 = atan2(Neck_y_for_L_shoulder, Neck_x_for_L_shoulder);
    // float LSAngle2 = atan2(L_Elbow_y_for_L_shoulder, L_Elbow_x_for_L_shoulder);
    // LSAngle = LSAngle1 + LSAngle2;
    // ROS_INFO_STREAM("Left shoulder is ");
    // ROS_INFO_STREAM(LSAngle);
    // //Left shoulder angle done

    // //calculating points for angle of left elbow
    // float L_wrist_y_for_L_elbow = LWrist.y - LElbow.y;
    // float L_wrist_x_for_L_elbow = LWrist.x - LElbow.x;

    // //Left elbow angle
    // float LEAngle1 = atan2(L_Elbow_y_for_L_shoulder, L_Elbow_x_for_L_shoulder);
    // float LEAngle2 = atan2(L_wrist_y_for_L_elbow, L_wrist_x_for_L_elbow);
    // LEAngle = LEAngle1 - LEAngle2;
    // ROS_INFO_STREAM("Left elbow is ");
    // ROS_INFO_STREAM(LEAngle);
  }
}

void parser()
{
  gestures = YAML::LoadFile("gestures.yaml");
  std::cout << gestures << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gesture_processer");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::spin();
  return 0;
}