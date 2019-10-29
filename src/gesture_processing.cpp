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

geometry_msgs::Twist msg;

void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
	if (msg.num_humans == 0)
	{
		command_received = false;
	}
  else
  {
    std::ofstream myfile;
    myfile.open("/home/igor/catkin_ws/src/openpose_gesture_control/src/kerneldata.txt",std::fstream::app);
    myfile << std::to_string(msg.human_list[0].body_key_points_with_prob[7].y);
    myfile << "\n"; 
    myfile.close();
    ROS_INFO_STREAM("I am writing!");
    if ((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      identifier = 1;
      command_received = true;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      identifier = 2;
      command_received = true;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y < msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y < msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      identifier = 5;
      command_received = true;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[2].y < msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y < msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      identifier = 6;
      command_received = true;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[2].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 40)) or ((msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[5].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 40)) and ((abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) > 50) or (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) > 50)))
    {
      identifier = 3;
      command_received = true;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y)) or ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y)))
    {
      identifier = 4;
      command_received = true;
    }
    else
    {
      identifier = 0;
      command_received = true;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
  ros::AsyncSpinner spinner(4);
	spinner.start();
	msg.linear.x = 0;
	msg.angular.z = 0;
	float poc = 0.7;

	while(ros::ok())
	{
		if (!command_received)
		{
			ROS_INFO_STREAM("There is no body in front of the camera!");
			float dvl = 0.0;
			float dva = 0.0;
   		msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
			msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
			//chatter_pub.publish(msg);
		}
		else
		{
      
			if (identifier == 1)
			{
				//geometry_msgs::Twist msg;
				float dvl = 0.4;
				float dva = -1.0;
   			msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
				msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 2)
			{
				//geometry_msgs::Twist msg;
				float dvl = 0.4;
				float dva = 1.0;
				msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
				msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 3)
			{
				//geometry_msgs::Twist msg;
				float dvl = -0.4;
				msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 4)
			{
				//geometry_msgs::Twist msg;
   			float dvl = 0.4;
				msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 5)
			{
				//geometry_msgs::Twist msg;
   			float dva = 1.0;
				msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 6)
			{
				//geometry_msgs::Twist msg;
   			float dva = -1.0;
				msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
				//chatter_pub.publish(msg);
			}
			else if (identifier == 0)
			{
				ROS_INFO_STREAM("Just waitin for the commands...");
				float dvl = 0.0;
				float dva = 0.0;
   			msg.linear.x = poc * msg.linear.x + (1 - poc) * dvl;
				msg.angular.z = poc * msg.angular.z + (1 - poc) * dva;
				//chatter_pub.publish(msg);
			}
		}		
		chatter_pub.publish(msg);
		//ROS_INFO_STREAM(msg);
		ros::Duration(0.05).sleep();
	}

  return 0;
}
