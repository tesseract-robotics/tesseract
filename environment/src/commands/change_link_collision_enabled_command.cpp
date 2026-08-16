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

namespace tesseract::environment
{
ChangeLinkCollisionEnabledCommand::ChangeLinkCollisionEnabledCommand()
  : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED)
{
}

ChangeLinkCollisionEnabledCommand::ChangeLinkCollisionEnabledCommand(common::LinkId link_id, bool enabled)
  : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED), link_id_(std::move(link_id)), enabled_(enabled)
{
}

const common::LinkId& ChangeLinkCollisionEnabledCommand::getLinkId() const { return link_id_; }
bool ChangeLinkCollisionEnabledCommand::getEnabled() const { return enabled_; }

bool ChangeLinkCollisionEnabledCommand::operator==(const ChangeLinkCollisionEnabledCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= link_id_ == rhs.link_id_;
  equal &= enabled_ == rhs.enabled_;
  return equal;
}
bool ChangeLinkCollisionEnabledCommand::operator!=(const ChangeLinkCollisionEnabledCommand& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::environment
