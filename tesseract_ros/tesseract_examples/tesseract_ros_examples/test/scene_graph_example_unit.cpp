/**
 * @file scene_graph_example_unit.cpp
 * @brief Runs the scene_graph_example
 *
 * @author Matthew Powelson
 * @date Feb 17, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/scene_graph_example.h>

using namespace tesseract_ros_examples;

TEST(TesseractROSExamples, SceneGraphExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  SceneGraphExample example(nh, false, false);
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scene_graph_example_unit");

  return RUN_ALL_TESTS();
}
