/**
 * @file pick_and_place_example.h
 * @brief An example of a robot picking up a box and placing it on a shelf.
 *
 * @author Mathew Powelson
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
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class PickAndPlaceExample : public Example
{
public:
  PickAndPlaceExample(const ros::NodeHandle& nh, bool plotting, bool rviz, int steps, bool write_to_file)
    : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file)
  {
  }
  ~PickAndPlaceExample() override = default;
  PickAndPlaceExample(const PickAndPlaceExample&) = default;
  PickAndPlaceExample& operator=(const PickAndPlaceExample&) = default;
  PickAndPlaceExample(PickAndPlaceExample&&) = default;
  PickAndPlaceExample& operator=(PickAndPlaceExample&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_PICK_AND_PLACE_EXAMPLE_H
