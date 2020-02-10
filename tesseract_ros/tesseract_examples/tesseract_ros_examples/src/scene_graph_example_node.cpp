/**
 * @file scene_graph_example_node.cpp
 * @brief Demonstrates manipulating the scene graph with a robot picking up another robot
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

#include <tesseract_ros_examples/scene_graph_example.h>

using namespace tesseract_ros_examples;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scene_graph_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool step_through = true;
  bool rviz = true;

  // Get ROS Parameters
  pnh.param("plotting", step_through, step_through);
  pnh.param("rviz", rviz, rviz);

  SceneGraphExample example(nh, step_through, rviz);
  example.run();
}
