/**
 * @file change_joint_origin_command.cpp
 * @brief Used to change joint origin
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
#include <tesseract/environment/commands/change_joint_origin_command.h>

#include <string>

namespace tesseract::environment
{
ChangeJointOriginCommand::ChangeJointOriginCommand() : Command(CommandType::CHANGE_JOINT_ORIGIN){};

// NOLINTNEXTLINE(modernize-pass-by-value)
ChangeJointOriginCommand::ChangeJointOriginCommand(std::string joint_name, const Eigen::Isometry3d& origin)
  : Command(CommandType::CHANGE_JOINT_ORIGIN), joint_name_(std::move(joint_name)), origin_(origin)
{
}

const std::string& ChangeJointOriginCommand::getJointName() const { return joint_name_; }
const Eigen::Isometry3d& ChangeJointOriginCommand::getOrigin() const { return origin_; }

bool ChangeJointOriginCommand::operator==(const ChangeJointOriginCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= origin_.isApprox(rhs.origin_, 1e-5);
  equal &= joint_name_ == rhs.joint_name_;
  return equal;
}
bool ChangeJointOriginCommand::operator!=(const ChangeJointOriginCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
