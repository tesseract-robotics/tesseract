/**
 * @file set_active_discrete_contact_manager_command.cpp
 * @brief Used to set the active discrete contact manager
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
#include <tesseract/environment/commands/set_active_discrete_contact_manager_command.h>

#include <string>

namespace tesseract::environment
{
SetActiveDiscreteContactManagerCommand::SetActiveDiscreteContactManagerCommand()
  : Command(CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER)
{
}

SetActiveDiscreteContactManagerCommand::SetActiveDiscreteContactManagerCommand(std::string active_contact_manager)
  : Command(CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER)
  , active_contact_manager_(std::move(active_contact_manager))
{
}

const std::string& SetActiveDiscreteContactManagerCommand::getName() const { return active_contact_manager_; }

bool SetActiveDiscreteContactManagerCommand::operator==(const SetActiveDiscreteContactManagerCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= active_contact_manager_ == rhs.active_contact_manager_;
  return equal;
}
bool SetActiveDiscreteContactManagerCommand::operator!=(const SetActiveDiscreteContactManagerCommand& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::environment
