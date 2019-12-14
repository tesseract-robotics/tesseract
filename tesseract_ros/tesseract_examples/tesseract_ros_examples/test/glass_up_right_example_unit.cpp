#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/glass_up_right_example.h>

using namespace tesseract_ros_examples;

TEST(TesseractROSExamples, GlassUpRightCppExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  GlassUpRightExample example(nh, false, false, 5, false, "cpp");
  EXPECT_TRUE(example.run());
}

TEST(TesseractROSExamples, GlassUpRightJsonExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  GlassUpRightExample example(nh, false, false, 5, false, "json");
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "glass_up_right_example_unit");

  return RUN_ALL_TESTS();
}
