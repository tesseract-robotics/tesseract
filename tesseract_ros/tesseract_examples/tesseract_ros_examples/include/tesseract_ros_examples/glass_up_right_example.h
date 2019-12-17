/**
 * @file glass_up_right_example.h
 * @brief An example of a robot with fixed orientation but free to move in cartesian space.
 *
 * @author Levi Armstrong
 * @date July 22, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{
/**
 * @brief An example of a robot with fixed orientation but free to move in cartesian space
 * leveraging tesseract and trajopt to generate a motion trajectory.
 */
class GlassUpRightExample : public Example
{
public:
  GlassUpRightExample(const ros::NodeHandle& nh,
                      bool plotting,
                      bool rviz,
                      int steps,
                      bool write_to_file,
                      std::string method)
    : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file), method_(std::move(method))
  {
  }
  ~GlassUpRightExample() override = default;
  GlassUpRightExample(const GlassUpRightExample&) = default;
  GlassUpRightExample& operator=(const GlassUpRightExample&) = default;
  GlassUpRightExample(GlassUpRightExample&&) = default;
  GlassUpRightExample& operator=(GlassUpRightExample&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
  std::string method_;

  trajopt::TrajOptProb::Ptr jsonMethod();
  trajopt::TrajOptProb::Ptr cppMethod();
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_EXAMPLE_H
