/**
 * @file remove_joint_command.cpp
 * @brief Used to remove a joint from the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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

#include <tesseract/common/utils.h>
#include <tesseract/environment/commands/remove_joint_command.h>

namespace tesseract::environment
{
RemoveJointCommand::RemoveJointCommand() : Command(CommandType::REMOVE_JOINT) {}

RemoveJointCommand::RemoveJointCommand(common::JointId joint_id)
  : Command(CommandType::REMOVE_JOINT), joint_id_(std::move(joint_id))
{
}

const common::JointId& RemoveJointCommand::getJointId() const { return joint_id_; }

bool RemoveJointCommand::operator==(const RemoveJointCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= joint_id_ == rhs.joint_id_;
  return equal;
}
bool RemoveJointCommand::operator!=(const RemoveJointCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
