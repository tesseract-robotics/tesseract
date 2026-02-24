/**
 * @file change_link_collision_enabled_command.cpp
 * @brief Used to change if a link is enabled for collision checking
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
#include <tesseract/environment/commands/change_link_collision_enabled_command.h>

#include <string>

namespace tesseract::environment
{
ChangeLinkCollisionEnabledCommand::ChangeLinkCollisionEnabledCommand()
  : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED)
{
}

ChangeLinkCollisionEnabledCommand::ChangeLinkCollisionEnabledCommand(std::string link_name, bool enabled)
  : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED), link_name_(std::move(link_name)), enabled_(enabled)
{
}

const std::string& ChangeLinkCollisionEnabledCommand::getLinkName() const { return link_name_; }
bool ChangeLinkCollisionEnabledCommand::getEnabled() const { return enabled_; }

bool ChangeLinkCollisionEnabledCommand::operator==(const ChangeLinkCollisionEnabledCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= link_name_ == rhs.link_name_;
  equal &= enabled_ == rhs.enabled_;
  return equal;
}
bool ChangeLinkCollisionEnabledCommand::operator!=(const ChangeLinkCollisionEnabledCommand& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::environment
