#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "convolution.h"
#include <iostream>
#include <fstream>

std::string command;
int identifier;
bool command_received = false;
float kernel[10] = {410.625763, 416.360229, 277.042542, 206.577835, 137.832367, 101.580055, 116.835854, 172.149567, 420.138153, 435.410248}; 
std::vector <float> intakeVec;
int kernelSize = sizeof(kernel);
int dataSize;
std::vector <float> outresultsVec;
bool result;
float intakeFloat[10];
float outresultsFloat[10];

void FromVec2Arr(std::vector <float> inVec, float* outFloat)
{
  int i;
  dataSize = sizeof(inVec);
  for(i = 0; i <= dataSize; ++i)
  {
    outFloat[i] = inVec[i];
  }
}

std::vector <float> FromArr2Vec(std::vector <float> outVec, float* inFloat)
{
  int i;
  dataSize = sizeof(inFloat);
  for(i = 0; i <= dataSize; ++i)
  {
    outVec[i] = inFloat[i];
  }
  return outVec;
}

bool convolve1D(float* in, float* out, int dataSize, float* kernel, int kernelSize)
{
    int i, j, k;

    // check validity of params
    if(!in || !out || !kernel) return false;
    if(dataSize <=0 || kernelSize <= 0) return false;

    // start convolution from out[kernelSize-1] to out[dataSize-1] (last)
    for(i = kernelSize-1; i < dataSize; ++i)
    {
        out[i] = 0;                             // init to 0 before accumulate

        for(j = i, k = 0; k < kernelSize; --j, ++k)
            out[i] += in[j] * kernel[k];
    }

    // convolution from out[0] to out[kernelSize-2]
    for(i = 0; i < kernelSize - 1; ++i)
    {
        out[i] = 0;                             // init to 0 before sum

        for(j = i, k = 0; j >= 0; --j, ++k)
            out[i] += in[j] * kernel[k];
    }

    return true;
}

geometry_msgs::Twist msg;
float values[10];
void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
	if (msg.num_humans > 0)
  {
    dataSize = sizeof(intakeVec);
    if (dataSize<10)
    {
      intakeVec.push_back(msg.human_list[0].body_key_points_with_prob[7].y);
    }
    else
    {
      intakeVec.erase(intakeVec.begin());
      intakeVec.push_back(msg.human_list[0].body_key_points_with_prob[7].y);
      FromVec2Arr(intakeVec,intakeFloat);
      result = convolve1D(intakeFloat, outresultsFloat,dataSize,kernel,kernelSize);
      if (result){
        ROS_INFO_STREAM(outresultsFloat);
      }
      else {
        ROS_INFO_STREAM("Oopps. Some problems happened.");
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