/**
 * @file scene_graph_example.cpp
 * @brief scene_graph_example implementation
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/scene_graph_example.h>
#include <tesseract_environment/core/utils.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM =
    "robot_description_semantic"; /**< Default ROS parameter for robot description */
const std::string ENVIRONMENT_TOPIC = "/monitored_environment";
const std::string JOINT_STATE_TOPIC = "/joint_states";

namespace tesseract_ros_examples
{
bool SceneGraphExample::run()
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;
  environment_monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(tesseract_, "environment_monitor");
  environment_monitor_->startStateMonitor(JOINT_STATE_TOPIC);
  environment_monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT,
                                                   ENVIRONMENT_TOPIC);

  environment_monitor_->getTesseract()->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example.dot");

  if (plotting_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ROS_INFO("Reconfiguring using moveJoint");
  }
  // Attach the iiwa to the end of the ABB using moveJoint. Notice that the joint transform stays the same. Only the
  // parent changes.
  environment_monitor_->getTesseract()->getEnvironment()->moveJoint("to_iiwa_mount", "tool0");

  // Save the scene graph to a file and publish the change
  environment_monitor_->getTesseract()->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example_moveJoint.dot");
  environment_monitor_->triggerEnvironmentUpdateEvent(environment_monitor_->UPDATE_ENVIRONMENT);

  if (plotting_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ROS_INFO("Reconfiguring using moveLink");
  }

  // Attach the iiwa to the end of the ABB using moveLink. The link to be moved is inferred to be the given Joint's
  // child
  tesseract_scene_graph::Joint new_joint("to_iiwa_mount");
  new_joint.parent_link_name = "tool0";
  new_joint.child_link_name = "iiwa_mount";
  new_joint.type = tesseract_scene_graph::JointType::FIXED;
  new_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  new_joint.parent_to_joint_origin_transform.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(0, 1, 0)));
  new_joint.parent_to_joint_origin_transform.translate(Eigen::Vector3d(0.15, 0.0, 0.0));
  environment_monitor_->getTesseract()->getEnvironment()->moveLink(std::move(new_joint));

  // Save the scene graph to a file and publish the change
  environment_monitor_->getTesseract()->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example_moveLink.dot");
  environment_monitor_->triggerEnvironmentUpdateEvent(environment_monitor_->UPDATE_ENVIRONMENT);

  if (plotting_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ROS_INFO("Open .dot files  in ~/.ros to see scene graph");
  }
  return true;
}
}  // namespace tesseract_ros_examples
