#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

std::string command;
int identifier;
bool command_received = false;

void chatterCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
{
	if (msg.num_humans == 0)
	{
		command_received = false;
	}
	else if ((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 30) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 30))
	{
		identifier = 1;
		command_received = true;
	}
	else if ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 40) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 40))
	{
		identifier = 2;
		command_received = true;
	}
	else if ((msg.human_list[0].body_key_points_with_prob[5].y < msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y < msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 30) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 30))
	{
		identifier = 5;
		command_received = true;
	}
	else if ((msg.human_list[0].body_key_points_with_prob[2].y < msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y < msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 30) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 30))
	{
		identifier = 6;
		command_received = true;
	}
	else if (((msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[2].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 20)) or ((msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[5].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 20)) and ((abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) > 50) or (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) > 50)))
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/openpose_ros/human_list", 1000, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); 
  ros::AsyncSpinner spinner(4);
	spinner.start();

	while(ros::ok())
	{
		if (command_received == true)
		{
			if (identifier == 1)
			{
				geometry_msgs::Twist msg;
   			msg.linear.x = 0.4;
				msg.angular.z = -1.0;
				chatter_pub.publish(msg);
			}
			else if (identifier == 2)
			{
				geometry_msgs::Twist msg;
   			msg.linear.x = 0.4;
				msg.angular.z = 1.0;
				chatter_pub.publish(msg);
			}
			else if (identifier == 3)
			{
				geometry_msgs::Twist msg;
   			msg.linear.x = -0.4;
				chatter_pub.publish(msg);
			}
			else if (identifier == 4)
			{
				geometry_msgs::Twist msg;
   			msg.linear.x = 0.4;
				chatter_pub.publish(msg);
			}
			else if (identifier == 5)
			{
				geometry_msgs::Twist msg;
   			msg.angular.z = 1.0;
				chatter_pub.publish(msg);
			}
			else if (identifier == 6)
			{
				geometry_msgs::Twist msg;
   			msg.angular.z = -1.0;
				chatter_pub.publish(msg);
			}
			else if (identifier == 0)
		{
			ROS_INFO_STREAM("Just waitin for the commands...");
		}
			command_received = false;
		}
		else
		{
			ROS_INFO_STREAM("There is no body in front of the camera!");
		}
	
		ros::Duration(0.1).sleep();
	}

  return 0;
}
