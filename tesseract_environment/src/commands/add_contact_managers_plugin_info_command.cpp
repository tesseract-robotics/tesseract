/**
 * @file add_contact_managers_plugin_info_command.cpp
 * @brief Used to add contact managers to the environment
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
#include <tesseract_environment/commands/add_contact_managers_plugin_info_command.h>

namespace tesseract_environment
{
AddContactManagersPluginInfoCommand::AddContactManagersPluginInfoCommand()
  : Command(CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO)
{
}

AddContactManagersPluginInfoCommand::AddContactManagersPluginInfoCommand(
    tesseract_common::ContactManagersPluginInfo contact_managers_plugin_info)
  : Command(CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO)
  , contact_managers_plugin_info_(std::move(contact_managers_plugin_info))
{
}

const tesseract_common::ContactManagersPluginInfo&
AddContactManagersPluginInfoCommand::getContactManagersPluginInfo() const
{
  return contact_managers_plugin_info_;
}

bool AddContactManagersPluginInfoCommand::operator==(const AddContactManagersPluginInfoCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= contact_managers_plugin_info_ == rhs.contact_managers_plugin_info_;
  return equal;
}
bool AddContactManagersPluginInfoCommand::operator!=(const AddContactManagersPluginInfoCommand& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract_environment
