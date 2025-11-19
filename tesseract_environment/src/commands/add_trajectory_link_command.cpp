/**
 * @file add_trajectory_link_command.cpp
 * @brief Used to add a  trajectory link to the environment
 *
 * @author Levi Armstrong
 * @date March 29, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#include <tesseract_environment/commands/add_trajectory_link_command.h>

#include <string>

namespace tesseract_environment
{
AddTrajectoryLinkCommand::AddTrajectoryLinkCommand() : Command(CommandType::ADD_TRAJECTORY_LINK) {}

AddTrajectoryLinkCommand::AddTrajectoryLinkCommand(std::string link_name,
                                                   std::string parent_link_name,
                                                   tesseract_common::JointTrajectory trajectory,
                                                   bool replace_allowed,
                                                   Method method)
  : Command(CommandType::ADD_TRAJECTORY_LINK)
  , link_name_(std::move(link_name))
  , parent_link_name_(std::move(parent_link_name))
  , trajectory_(std::move(trajectory))
  , replace_allowed_(replace_allowed)
  , method_(method)
{
}

const std::string& AddTrajectoryLinkCommand::getLinkName() const { return link_name_; }
const std::string& AddTrajectoryLinkCommand::getParentLinkName() const { return parent_link_name_; }
const tesseract_common::JointTrajectory& AddTrajectoryLinkCommand::getTrajectory() const { return trajectory_; }
bool AddTrajectoryLinkCommand::replaceAllowed() const { return replace_allowed_; }
AddTrajectoryLinkCommand::Method AddTrajectoryLinkCommand::getMethod() const { return method_; }

bool AddTrajectoryLinkCommand::operator==(const AddTrajectoryLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= (link_name_ == rhs.link_name_);
  equal &= (parent_link_name_ == rhs.parent_link_name_);
  equal &= (trajectory_ == rhs.trajectory_);
  equal &= (replace_allowed_ == rhs.replace_allowed_);
  equal &= (method_ == rhs.method_);
  return equal;
}
bool AddTrajectoryLinkCommand::operator!=(const AddTrajectoryLinkCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_environment
