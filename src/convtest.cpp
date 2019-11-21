#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>

std::string command;
int identifier;
bool command_received = false;
std::vector <float> intake = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};






std::vector<float> convolution(std::vector<float> intake, std::vector<float> kernel)
{
  int i, j, k;
  std::vector<float> out = {0.0};
  int dataSize = intake.size();
  int kernelSize = kernel.size();
  if (dataSize == 0 || kernelSize == 0 || dataSize <= kernelSize)
  {
    return out;
  }
  for(i = kernelSize-1; i < dataSize; ++i)
  {
    out[i] = 0;

    for(j = i, k = 0; k < kernelSize; --j, ++k)
    {
      out[i] += intake[j] * kernel[k];
    }
  }

  for (i = 0; i < kernelSize - 1; ++i)
  {
    out[i] = 0;

    for(j = i, k = 0; j >= 0; --j, ++k)
    {
      out[i] += intake[j] * kernel[k];
    }
  }

  return out;
}

geometry_msgs::Twist msg;
void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
  std::vector <float> kernel = {410.625763, 416.360229, 277.042542, 206.577835, 137.832367, 101.580055, 116.835854, 172.149567, 420.138153, 435.410248};
  int kernelSize = kernel.size();
	if (msg.num_humans > 0)
  {
    int dataSize = sizeof(intake);
    if (dataSize<20)
    {
      intake.push_back(msg.human_list[0].body_key_points_with_prob[7].y);
    }
    else
    {
      intake.erase(intake.begin());
      intake.push_back(msg.human_list[0].body_key_points_with_prob[7].y);
      std::vector <float> result = convolution(intake,kernel);
      if (result.size() > 0){
        int s = result.size();
        for (int i; i<=s; ++i)
        ROS_INFO_STREAM(result[i]);
      }
      else {
        ROS_INFO_STREAM("Oops. Some problems happened.");
      }
    }
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::spin();
  return 0;
}
