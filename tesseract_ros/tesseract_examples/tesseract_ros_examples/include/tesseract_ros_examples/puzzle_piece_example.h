#ifndef TESSERACT_ROS_EXAMPLES_PUZZLE_PIECE_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_PUZZLE_PIECE_EXAMPLE_H

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

class PuzzlePieceExample : public Example
{
public:
  PuzzlePieceExample(ros::NodeHandle nh, bool plotting, bool rviz) : Example(plotting, rviz), nh_(nh) {}
  ~PuzzlePieceExample() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;

  trajopt::ProblemConstructionInfo cppMethod();

  tesseract_common::VectorIsometry3d makePuzzleToolPoses();
};

}
#endif // TESSERACT_ROS_EXAMPLES_PUZZLE_PIECE_EXAMPLE_H
