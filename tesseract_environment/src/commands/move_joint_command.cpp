/**
 * @file move_joint_command.cpp
 * @brief Used to move a link in the environment
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

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/move_joint_command.h>

#include <string>

namespace tesseract::environment
{
MoveJointCommand::MoveJointCommand() : Command(CommandType::MOVE_JOINT) {}

MoveJointCommand::MoveJointCommand(std::string joint_name, std::string parent_link)
  : Command(CommandType::MOVE_JOINT), joint_name_(std::move(joint_name)), parent_link_(std::move(parent_link))
{
}

const std::string& MoveJointCommand::getJointName() const { return joint_name_; }
const std::string& MoveJointCommand::getParentLink() const { return parent_link_; }

bool MoveJointCommand::operator==(const MoveJointCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= joint_name_ == rhs.joint_name_;
  equal &= parent_link_ == rhs.parent_link_;
  return equal;
}
bool MoveJointCommand::operator!=(const MoveJointCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
