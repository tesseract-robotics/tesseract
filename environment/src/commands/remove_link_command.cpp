/**
 * @file remove_link_command.cpp
 * @brief Used to remove a link from the environment
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
#include <tesseract/environment/commands/remove_link_command.h>

namespace tesseract::environment
{
RemoveLinkCommand::RemoveLinkCommand() : Command(CommandType::REMOVE_LINK) {}

RemoveLinkCommand::RemoveLinkCommand(common::LinkId link_id)
  : Command(CommandType::REMOVE_LINK), link_id_(std::move(link_id))
{
}

const common::LinkId& RemoveLinkCommand::getLinkId() const { return link_id_; }

bool RemoveLinkCommand::operator==(const RemoveLinkCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= link_id_ == rhs.link_id_;
  return equal;
}
bool RemoveLinkCommand::operator!=(const RemoveLinkCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
