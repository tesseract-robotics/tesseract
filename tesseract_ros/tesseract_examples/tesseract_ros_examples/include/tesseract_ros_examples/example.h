/**
 * @file examples.h
 * @brief Examples base class
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ROS_EXAMPLES_EXAMPLES_H
#define TESSERACT_ROS_EXAMPLES_EXAMPLES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

namespace tesseract_ros_examples
{

class Example
{
public:
  Example(bool plotting, bool rviz) : plotting_(plotting), rviz_(rviz), tesseract_(std::make_shared<tesseract::Tesseract>()) {}
  virtual ~Example() = default;

  virtual bool run() = 0;

protected:
  bool plotting_;
  bool rviz_;
  tesseract::Tesseract::Ptr tesseract_;
  ros::ServiceClient modify_env_rviz_;
  ros::ServiceClient get_env_changes_rviz_;

  bool checkRviz()
  {

    // Get the current state of the environment.
    // Usually you would not be getting environment state from rviz
    // this is just an example. You would be gettting it from the
    // environment_monitor node. Need to update examples to launch
    // environment_monitor node.
    get_env_changes_rviz_.waitForExistence();
    tesseract_msgs::GetEnvironmentChanges env_changes;
    env_changes.request.revision = 0;
    if (get_env_changes_rviz_.call(env_changes))
    {
      ROS_INFO("Retrieve current environment changes!");
    }
    else
    {
      ROS_ERROR("Failed to retrieve current environment changes!");
      return false;
    }

    // There should not be any changes but check
    if (env_changes.response.revision != 0)
    {
      ROS_ERROR("The environment has changed externally!");
      return false;
    }
    return true;
  }

  /**
   * @brief Send RViz the latest number of commands
   * @param n The past revision number
   * @return True if successful otherwise false
   */
  bool sendRvizChanges(unsigned long past_revision)
  {
    modify_env_rviz_.waitForExistence();
    tesseract_msgs::ModifyEnvironment update_env;
    update_env.request.id = tesseract_->getEnvironment()->getName();
    update_env.request.revision = past_revision;
    if (!tesseract_rosutils::toMsg(
            update_env.request.commands, tesseract_->getEnvironment()->getCommandHistory(), update_env.request.revision))
    {
      ROS_ERROR("Failed to generate commands to update rviz environment!");
      return false;
    }

    if (modify_env_rviz_.call(update_env))
    {
      ROS_INFO("RViz environment Updated!");
    }
    else
    {
      ROS_INFO("Failed to update rviz environment");
      return false;
    }

    return true;
  }
};

}
#endif // TESSERACT_ROS_EXAMPLES_EXAMPLES_H
