/**
 * @file add_contact_managers_plugin_info_command.h
 * @brief Used to add contact managers plugin info to the environment
 *
 * @author Levi Armstrong
 * @date October 26, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_ADD_CONTACT_MANAGERS_PLUGIN_INFO_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_CONTACT_MANAGERS_PLUGIN_INFO_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_common/types.h>

namespace tesseract_environment
{
class AddContactManagersPluginInfoCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddContactManagersPluginInfoCommand>;
  using ConstPtr = std::shared_ptr<const AddContactManagersPluginInfoCommand>;

  /**
   * @brief Add contact manager plugins
   * @param contact_managers_plugin_info Contact managers plugin information
   */
  AddContactManagersPluginInfoCommand(tesseract_common::ContactManagersPluginInfo contact_managers_plugin_info)
    : contact_managers_plugin_info_(std::move(contact_managers_plugin_info))
  {
  }

  CommandType getType() const final { return CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO; }
  const tesseract_common::ContactManagersPluginInfo& getContactManagersPluginInfo() const
  {
    return contact_managers_plugin_info_;
  }

private:
  tesseract_common::ContactManagersPluginInfo contact_managers_plugin_info_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ADD_CONTACT_MANAGERS_PLUGIN_INFO_COMMAND_H
