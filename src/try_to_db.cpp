//#include "ros/ros.h"
//#include "openpose_ros_msgs/OpenPoseHumanList.h"
//#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "convolution.h"
#include <iostream>
#include <fstream>
#include <vector>

float kernel[10] = {410.625763, 416.360229, 277.042542, 206.577835, 137.832367, 101.580055, 116.835854, 172.149567, 420.138153, 435.410248};

void FromVec2Arr(std::vector <float> inVec, float* outFloat)
{
  int i;
  int dataSize = sizeof(inVec);
  for(i = 0; i <= dataSize; ++i)
  {
    outFloat[i] = inVec[i];
  }
}


// Seg fault is here
std::vector <float> FromArr2Vec(std::vector <float> outVec, float* inFloat)
{
  int i;
  int dataSize = sizeof(inFloat);
  for(i = 0; i <= dataSize; ++i)
  {
    outVec[i] = inFloat[i];
  }
  return outVec;
}