/**
 * @file replace_joint_command.h
 * @brief Used to replace joint in environment
 *
 * @author Levi Armstrong
 * @date February 11, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_REPLACE_JOINT_COMMAND_H
#define TESSERACT_ENVIRONMENT_REPLACE_JOINT_COMMAND_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_scene_graph/joint.h>

namespace tesseract_environment
{
class ReplaceJointCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ReplaceJointCommand>;
  using ConstPtr = std::shared_ptr<const ReplaceJointCommand>;

  /**
   * @brief Replace a joint in the environment
   *
   * If the joint does not exist:
   *
   *        This command should result in an error
   *
   * If the child link is not the same:
   *
   *        This command should result in an error
   *
   * @param joint The joint to be replaced
   */
  ReplaceJointCommand(const tesseract_scene_graph::Joint& joint)
    : joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
  {
    if (joint_->type != tesseract_scene_graph::JointType::FIXED)
    {
      //      if ()
      /** @todo check limits */
    }
  }

  CommandType getType() const final { return CommandType::REPLACE_JOINT; }
  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }

private:
  tesseract_scene_graph::Joint::ConstPtr joint_;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_REPLACE_JOINT_COMMAND_H
