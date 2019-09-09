#include "ros/ros.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "std_msgs/Int16.h"
#include <cstdlib>

/**
 * @brief Gesture recognizer ROS wrapper class
 * 
 */
class GestureRecognizer
{
public:
  /**
   * @brief Construct a new Gesture Recognizer object
   * 
   */
  GestureRecognizer()
  {
    human_list_sub_ = nh_.subscribe("human_list", 1, &GestureRecognizer::humanListCallback, this);
    gesture_pub_ = nh_.advertise<std_msgs::Int16>("detected_gesture", 1);
    ROS_INFO("Gesture Recognizer is good to go");
  }

private:
  /**
   * @brief Callback for human skeletal model message
   * 
   * @param msg 
   */
  void humanListCallback(const openpose_ros_msgs::OpenPoseHumanList& msg)
  {
    /*
     * Try to detect a gesture
     */
    if (msg.num_humans == 0)
    {
      // No humans were detected
      gesture_code_msg_.data = -1;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      gesture_code_msg_.data = 1;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      gesture_code_msg_.data = 2;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y < msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y < msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      gesture_code_msg_.data = 5;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[2].y < msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y < msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      gesture_code_msg_.data = 6;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[2].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 40)) or ((msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[5].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 40)) and ((abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) > 50) or (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) > 50)))
    {
      gesture_code_msg_.data = 3;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y)) or ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y)))
    {
      gesture_code_msg_.data = 4;
    }
    else
    {
      // No recognizable commands
      gesture_code_msg_.data = 0;
    }

    /*
     * Publish the gesture message
     */
    gesture_pub_.publish(gesture_code_msg_);
  }

  // Declare members
  ros::NodeHandle nh_;
  ros::Subscriber human_list_sub_;
  ros::Publisher gesture_pub_;
  std_msgs::Int16 gesture_code_msg_;
};

/*
 * Main
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  GestureRecognizer gesture_recognizer;
  ros::spin();

  return 0;
}
