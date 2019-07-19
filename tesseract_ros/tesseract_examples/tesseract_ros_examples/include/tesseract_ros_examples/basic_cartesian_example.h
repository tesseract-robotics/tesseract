#ifndef TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{

class BasicCartesianExample : public Example
{
public:
  BasicCartesianExample(ros::NodeHandle nh, bool plotting, bool rviz, int steps, std::string method) : Example(plotting, rviz), nh_(nh), steps_(steps), method_(method) {}
  ~BasicCartesianExample() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  std::string method_;

  trajopt::TrajOptProb::Ptr jsonMethod();
  trajopt::TrajOptProb::Ptr cppMethod();

  bool addPointCloud();
};

}

#endif // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
