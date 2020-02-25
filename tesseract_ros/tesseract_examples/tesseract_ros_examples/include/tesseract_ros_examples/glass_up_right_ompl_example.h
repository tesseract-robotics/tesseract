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
#ifndef TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_OMPL_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_OMPL_EXAMPLE_H

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
 * leveraging OMPL RRTConnect to generate a motion trajectory.
 */
class GlassUpRightOMPLExample : public Example
{
public:
  GlassUpRightOMPLExample(const ros::NodeHandle& nh,
                          bool plotting,
                          bool rviz,
                          double range,
                          bool use_constraint,
                          bool use_trajopt_constraint,
                          double planning_time);
  ~GlassUpRightOMPLExample() override = default;
  GlassUpRightOMPLExample(const GlassUpRightOMPLExample&) = default;
  GlassUpRightOMPLExample& operator=(const GlassUpRightOMPLExample&) = default;
  GlassUpRightOMPLExample(GlassUpRightOMPLExample&&) = default;
  GlassUpRightOMPLExample& operator=(GlassUpRightOMPLExample&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  double range_;
  bool use_constraint_;
  bool use_trajopt_constraint_;
  double planning_time_;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_GLASS_UP_RIGHT_OMPL_EXAMPLE_H
