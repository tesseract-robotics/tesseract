#ifndef TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{

class PickAndPlaceExample : public Example
{
public:
  PickAndPlaceExample(ros::NodeHandle nh, bool plotting, bool rviz, int steps, bool write_to_file) : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file) {}
  ~PickAndPlaceExample() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;

};

}

#endif // TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H
