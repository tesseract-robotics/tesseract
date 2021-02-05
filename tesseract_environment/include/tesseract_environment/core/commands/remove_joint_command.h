/**
 * @file remove_joint_command.h
 * @brief Used to remove joint from environment
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
#ifndef TESSERACT_ENVIRONMENT_REMOVE_JOINT_COMMAND_H
#define TESSERACT_ENVIRONMENT_REMOVE_JOINT_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>

namespace tesseract_environment
{
class RemoveJointCommand : public Command
{
public:
  using Ptr = std::shared_ptr<RemoveJointCommand>;
  using ConstPtr = std::shared_ptr<const RemoveJointCommand>;

  /**
   * @brief Removes a joint from the environment
   *
   *        All child components (links/joints) should be removed
   *
   * @param name Name of the joint to be removed
   */
  RemoveJointCommand(std::string joint_name) : joint_name_(std::move(joint_name)) {}

  CommandType getType() const final { return CommandType::REMOVE_JOINT; }
  const std::string& getJointName() const { return joint_name_; }

private:
  std::string joint_name_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_REMOVE_JOINT_COMMAND_H
