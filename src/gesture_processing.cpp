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
  : message_counter_(0)
  , max_count_(20)

  {
    human_list_sub_ = nh_.subscribe("human_list", 1, &GestureRecognizer::humanListCallback, this);
    gesture_pub_ = nh_.advertise<std_msgs::Int16>("detected_gesture", 1);
    command_history_.clear();
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
    message_counter_++;
    if (msg.num_humans == 0)
    {
      // No humans were detected
      command_history_[-1]++;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      // Forward + right
      command_history_[2]++;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      // Forward + left
      command_history_[4]++;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[5].y < msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y < msg.human_list[0].body_key_points_with_prob[7].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 50))
    {
      // Left
      command_history_[5]++;
    }
    else if ((msg.human_list[0].body_key_points_with_prob[2].y < msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y < msg.human_list[0].body_key_points_with_prob[4].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) < 50) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 50))
    {
      // Right
      command_history_[1]++;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[2].y) and (abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[4].y) < 40)) or ((msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[5].y) and (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[7].y) < 40)) and ((abs(msg.human_list[0].body_key_points_with_prob[2].y - msg.human_list[0].body_key_points_with_prob[3].y) > 50) or (abs(msg.human_list[0].body_key_points_with_prob[5].y - msg.human_list[0].body_key_points_with_prob[6].y) > 50)))
    {
      // Back
      command_history_[6]++;
    }
    else if (((msg.human_list[0].body_key_points_with_prob[2].y > msg.human_list[0].body_key_points_with_prob[3].y) and (msg.human_list[0].body_key_points_with_prob[3].y > msg.human_list[0].body_key_points_with_prob[4].y)) or ((msg.human_list[0].body_key_points_with_prob[5].y > msg.human_list[0].body_key_points_with_prob[6].y) and (msg.human_list[0].body_key_points_with_prob[6].y > msg.human_list[0].body_key_points_with_prob[7].y)))
    {
      // Forward
      command_history_[3]++;
    }
    else
    {
      // No recognizable commands
      command_history_[0]++;
    }

    /*
     * Publish the gesture message if sufficient amount of data was gathered
     */
    if (message_counter_ >= max_count_)
    {
      gesture_code_msg_.data = getCode();
      gesture_pub_.publish(gesture_code_msg_);
      message_counter_ = 0;
    }
  }

  int getCode()
  {
    unsigned int max = 0;
    int command = -1;
    for (const auto& cmd_pair : command_history_)
    {
      if (cmd_pair.second > max)
      {
        command = cmd_pair.first;
        max = cmd_pair.second;
      }
    }
    command_history_.clear();
    return command;
  }

  // Declare members
  ros::NodeHandle nh_;
  ros::Subscriber human_list_sub_;
  ros::Publisher gesture_pub_;
  std_msgs::Int16 gesture_code_msg_;

  std::map<int, unsigned int> command_history_;
  unsigned int message_counter_;
  const unsigned int max_count_;
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
