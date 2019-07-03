/*
 * robot_communication_monitor.cpp
 *
 * Checks for fanuc communication errors
 *
 * Created on June 14, 2019
 *    Author: rwall
 *
 *
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "boost/bind.hpp"
#include <sensor_msgs/JointState.h>
#include <tesseract_monitoring/current_state_monitor.h>

// Test run commands:
// rosrun tesseract_monitoring robot_communication_monitor_node
// rostopic pub -r 1 /joint_states sensor_msgs/JointState
// rostopic echo /fanuc_com_error

class FanucCommError
{
public:
  FanucCommError(ros::NodeHandle& nh)
  {

    fanuc_comm_error.data = false;
    chatter_pub = nh.advertise<std_msgs::Bool>("fanuc_com_error", 20);

    chatter_sub = nh.subscribe("joint_states", 20,
                               &FanucCommError::subscriberCallback, this);
  }
  void startTimer()
  {
    timer = FanucCommError::nh.createTimer(
          ros::Duration(error_timer), &FanucCommError::timerCallback, this);
  }

  void subscriberCallback(const sensor_msgs::JointStateConstPtr& joint_state)
  {
    count = count + 1; // increase count of joint states subscription

    // ROS_INFO("\n subscriber called. Count is %d \n", count); //debugging
    // statement
  }

  void timerCallback(const ros::TimerEvent&)
  {
    // ROS_INFO("\n Timer called. Count is %d",count); //debugging statement

    if (count < 1)
    {
      fanuc_comm_error.data = true;
    }

    else
    {
      fanuc_comm_error.data = false;
    }

    count = 0;                             // re-initialize error check
    chatter_pub.publish(fanuc_comm_error); // publish error status

    // ROS_INFO("FANUC COM ERROR= %s \n", fanuc_comm_error.data ? "true" :
    // "false");//debugging statement
  }

private:
  ros::Timer timer;
  ros::NodeHandle nh;
  ros::Publisher chatter_pub;
  ros::Subscriber chatter_sub;

  std_msgs::Bool fanuc_comm_error; // fanuc_comm_error msg

  int count = 0;       // subscriber count
  int error_timer = 3; // check for connection (Fanuc Comm Error) every 3
  // seconds
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_communication_monitor_node");
  ros::NodeHandle nh;
  FanucCommError FCE(nh);

  ros::Rate loop_rate(10); // run at 10 Hz

  while (ros::ok())
  {

    FCE.startTimer();
    ros::spin();

    loop_rate.sleep();
  }

  return 0;
}
