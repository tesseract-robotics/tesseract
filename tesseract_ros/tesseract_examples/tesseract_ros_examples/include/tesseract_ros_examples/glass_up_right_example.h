#ifndef TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H
#include <tesseract_common/macros.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{

class GlassUpRightExample : public Example
{
public:
  GlassUpRightExample(ros::NodeHandle nh, bool plotting, bool rviz, int steps, bool write_to_file, std::string method) : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file), method_(method) {}
  ~GlassUpRightExample() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
  std::string method_;

  trajopt::TrajOptProb::Ptr jsonMethod();
  trajopt::TrajOptProb::Ptr cppMethod();

};

}

#endif // TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H
