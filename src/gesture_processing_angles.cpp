#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "math.h"
#include <vector>
#include "yaml-cpp/yaml.h"

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
// Left hand
std::vector <float> Neck_LShoulder = {0.0, 0.0};
std::vector <float> LElbow_LShoulder = {0.0, 0.0};
std::vector <float> LShoulder_LElbow = {0.0, 0.0};
std::vector <float> LWrist_LElbow = {0.0, 0.0};

//Here all 4 angles are stored
std::vector <float> angles = {0.0, 0.0, 0.0, 0.0};
//Order: RShoulder, RElbow, LShoulder, LElbow

//for later use
std::vector <std::vector <float> > ref_angles_for_pose;
std::vector <std::vector <float> > ref_error_for_pose;
std::vector <std::string> pose_names;

//For yaml parser
YAML::Node gestures;

//info about all needed keypoints
openpose_ros_msgs::PointWithProb Neck, RShoulder, RElbow, RWrist, LShoulder, LElbow, LWrist;

float remap(float value, float start1, float end1, float start2, float end2)
{
  float outgoing = start2 + (end2 - start2) * ((value - start1) / (end1 - start1));
  return outgoing;
}

void Pose_to_angle(const openpose_ros_msgs::OpenPoseHumanList& pose_msg)
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
      angles[0] = atan2(Neck_RShoulder[0]*RElbow_RShoulder[0] + Neck_RShoulder[1]*RElbow_RShoulder[1], Neck_RShoulder[0]*RElbow_RShoulder[1] - Neck_RShoulder[1]*RElbow_RShoulder[0]);

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
      angles[2] = atan2(Neck_LShoulder[0]*LElbow_LShoulder[0] + Neck_LShoulder[1]*LElbow_LShoulder[1], Neck_LShoulder[0]*LElbow_LShoulder[1] - Neck_LShoulder[1]*LElbow_LShoulder[0]);

      angles[3] = atan2(LShoulder_LElbow[0]*LWrist_LElbow[0] + LShoulder_LElbow[1]*LWrist_LElbow[1],LShoulder_LElbow[0]*LWrist_LElbow[1] - LShoulder_LElbow[1]*LWrist_LElbow[0]); 
    }
    for (int i = 0; i < angles.size(); i++)
    {
      angles[i] = angles[i] * 180/M_PI;
      angles[i] = remap(angles[i], -180, 180, 0, 360);
    }
    std::cout << "RShoulder: " << angles[0] << std::endl << "RElbow: " << angles[1] << std::endl << "LShoulder: " << angles[2] << std::endl << "LElbow: " << angles[3] << std::endl;
  }
}

void parser_init()
{
  std::cout << "Angle 1 - Right shoulder" << std::endl << "Angle 2 - Right elbow" << std::endl;
  std::cout << "Angle 3 - Left shoulder" << std::endl << "Angle 4 - Left elbow" << std::endl;
  ros::Duration(5).sleep();
  gestures = YAML::LoadFile("/home/igor/catkin_ws/src/openpose_gesture_control/src/gestures.yaml");
  gestures = gestures["gestures"];
  for (std::size_t i=0;i<gestures.size();i++)
  {
    std::vector <float > pose_vec = {0.0, 0.0, 0.0, 0.0};
    std::vector <float > error_vec = {0.0, 0.0, 0.0, 0.0};
    pose_vec[0] = gestures[i]["right_shoulder"]["angle"].as<float>();
    pose_vec[1] = gestures[i]["right_elbow"]["angle"].as<float>();
    pose_vec[2] = gestures[i]["left_shoulder"]["angle"].as<float>();
    pose_vec[3] = gestures[i]["left_elbow"]["angle"].as<float>();

    error_vec[0] = gestures[i]["right_shoulder"]["error"].as<float>();
    error_vec[1] = gestures[i]["right_elbow"]["error"].as<float>();
    error_vec[2] = gestures[i]["left_shoulder"]["error"].as<float>();
    error_vec[3] = gestures[i]["left_elbow"]["error"].as<float>();

    pose_names.push_back(gestures[i]["gesture_name"].as<std::string>());
    ref_angles_for_pose.push_back(pose_vec);
    ref_error_for_pose.push_back(error_vec);
  }
  for (int num_pose = 0; num_pose < ref_angles_for_pose.size(); num_pose++)
  {
    std::cout << "Command name: " << pose_names[num_pose] << std::endl;
    for (int num_angle = 0; num_angle < ref_angles_for_pose[num_pose].size(); num_angle++)
    {
      std::cout << "Angle " << num_angle+1 << ": " << ref_angles_for_pose[num_pose][num_angle] << std::endl;
      std::cout << "Possible error of angle " << num_angle+1 << " for you to make: " << ref_error_for_pose[num_pose][num_angle] << std::endl;
    }
  }
  std::cout << "Parsing done!" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gesture_processer");
  ros::NodeHandle nh;
  parser_init();
  ros::Duration(2).sleep();
  std::cout << "I am ready!" << std::endl;
  ros::Subscriber sub = nh.subscribe("openpose_ros/human_list", 1, Pose_to_angle);
  ros::Publisher pub = nh.advertise<std_msgs::String>("openpose_ros/found_pose", 1);
  while(ros::ok)
  {
    ros::spinOnce();
    std::string final_pose = "none";
    std_msgs::String pose_msg;
    float sum_of_errors = 1500.0; //just some big value to make sure any valid pose will be recogised

    //checking ol poses which were parsed from yaml file
    for (int pose_num = 0; pose_num < ref_angles_for_pose.size(); pose_num++)
    {
      float errors_in_loop = 0.0; //to count total error made by user to find later the pose with highest probability
      int empty_angles = 0; //to make total error calculation more fair later
      for (int angle_num = 0; angle_num < angles.size(); angle_num++)
      {
        if (ref_error_for_pose[pose_num][angle_num] == 360)
        {
          empty_angles++;
          continue;
        }
        float error = fabs(ref_angles_for_pose[pose_num][angle_num] - angles[angle_num]);
        if (error >= 180.0)
        {
          error = 360.0 - error;
        }
        // check if user error is in allowed range if no, ignore the whole pose and reset total error variable, else add this to total error of the pose for later use
        if (ref_error_for_pose[pose_num][angle_num] < error)
        {
          errors_in_loop = 0.0;
          break;
        }
        else
        {
          errors_in_loop += error;
        }
      }
      //TODO add check of empty_angles variable to prevent division by 0 (maybe done)
      if (empty_angles == 4)
      {
        if (final_pose == "none")
        {
          final_pose = pose_names[pose_num];
        }
      }
      else if ((sum_of_errors > errors_in_loop/(4.0-empty_angles)) and (errors_in_loop != 0.0))
      {
        sum_of_errors = errors_in_loop/(4-empty_angles);
        final_pose = pose_names[pose_num];
      }
    }
    std::cout << final_pose << std::endl;
    pose_msg.data = final_pose;
    pub.publish(pose_msg);
    ros::Rate(10).sleep();
  }
  return 0;
}