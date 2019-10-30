#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include <cstdlib>
#include <iostream>
#include <fstream>

void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
	if (msg.num_humans > 0)
  {
    std::ofstream myfile;
    myfile.open("/home/igor/catkin_ws/src/openpose_gesture_control/src/kerneldata.txt",std::fstream::app);
    myfile << std::to_string(msg.human_list[0].body_key_points_with_prob[7].y);
    myfile << "\n"; 
    myfile.close();
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