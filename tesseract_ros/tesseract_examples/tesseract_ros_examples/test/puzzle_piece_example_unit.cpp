#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/puzzle_piece_example.h>

using namespace tesseract_ros_examples;

TEST(TesseractROSExamples, PuzzlePieceCppExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  PuzzlePieceExample example(nh, false, false);
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "puzzle_piece_example_unit");

  return RUN_ALL_TESTS();
}
