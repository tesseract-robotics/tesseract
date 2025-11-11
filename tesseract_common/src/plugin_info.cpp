/**
 * @file plugin_info.cpp
 * @brief Common Tesseract Plugin Infos
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>

namespace tesseract_common
{
/*********************************************************/
/******               PluginInfo                     *****/
/*********************************************************/

std::string PluginInfo::getConfigString() const { return toYAMLString(config); }

bool PluginInfo::operator==(const PluginInfo& rhs) const
{
  bool equal = true;
  equal &= class_name == rhs.class_name;
  equal &= compareYAML(config, rhs.config);

  return equal;
}
bool PluginInfo::operator!=(const PluginInfo& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******           PluginInfoContainer                *****/
/*********************************************************/

void PluginInfoContainer::clear()
{
  default_plugin.clear();
  plugins.clear();
}

bool PluginInfoContainer::operator==(const PluginInfoContainer& rhs) const
{
  bool equal = true;
  equal &= default_plugin == rhs.default_plugin;
  equal &= isIdenticalMap<PluginInfoMap, PluginInfo>(plugins, rhs.plugins);

  return equal;
}
bool PluginInfoContainer::operator!=(const PluginInfoContainer& rhs) const { return !operator==(rhs); }

/*********************************************************/
/**********          ProfilePluginInfo           *********/
/*********************************************************/
void ProfilesPluginInfo::insert(const ProfilesPluginInfo& other)
{
  search_paths.insert(search_paths.end(), other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(search_libraries.end(), other.search_libraries.begin(), other.search_libraries.end());

  for (const auto& group_plugins : other.plugin_infos)
  {
    for (const auto& plugin_info : group_plugins.second)
      plugin_infos[group_plugins.first][plugin_info.first] = plugin_info.second;
  }
}

void ProfilesPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  plugin_infos.clear();
}

bool ProfilesPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && plugin_infos.empty());
}

bool ProfilesPluginInfo::operator==(const ProfilesPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdentical<std::string>(search_paths, rhs.search_paths);
  equal &= isIdentical<std::string>(search_libraries, rhs.search_libraries);
  equal &= isIdenticalMap<std::map<std::string, PluginInfoMap>, PluginInfoMap>(plugin_infos, rhs.plugin_infos);
  return equal;
}
bool ProfilesPluginInfo::operator!=(const ProfilesPluginInfo& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******           KinematicsPluginInfo               *****/
/*********************************************************/
void KinematicsPluginInfo::insert(const KinematicsPluginInfo& other)
{
  search_paths.insert(search_paths.end(), other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(search_libraries.end(), other.search_libraries.begin(), other.search_libraries.end());

  for (const auto& group_plugins : other.fwd_plugin_infos)
  {
    if (!group_plugins.second.default_plugin.empty())
      fwd_plugin_infos[group_plugins.first].default_plugin = group_plugins.second.default_plugin;

    for (const auto& plugin_info : group_plugins.second.plugins)
      fwd_plugin_infos[group_plugins.first].plugins[plugin_info.first] = plugin_info.second;
  }

  for (const auto& group_plugins : other.inv_plugin_infos)
  {
    if (!group_plugins.second.default_plugin.empty())
      inv_plugin_infos[group_plugins.first].default_plugin = group_plugins.second.default_plugin;

    for (const auto& plugin_info : group_plugins.second.plugins)
      inv_plugin_infos[group_plugins.first].plugins[plugin_info.first] = plugin_info.second;
  }
}

void KinematicsPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  fwd_plugin_infos.clear();
  inv_plugin_infos.clear();
}

bool KinematicsPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && fwd_plugin_infos.empty() && inv_plugin_infos.empty());
}

bool KinematicsPluginInfo::operator==(const KinematicsPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdentical<std::string>(search_paths, rhs.search_paths);
  equal &= isIdentical<std::string>(search_libraries, rhs.search_libraries);
  equal &= isIdenticalMap<std::map<std::string, PluginInfoContainer>, PluginInfoContainer>(fwd_plugin_infos,
                                                                                           rhs.fwd_plugin_infos);
  equal &= isIdenticalMap<std::map<std::string, PluginInfoContainer>, PluginInfoContainer>(inv_plugin_infos,
                                                                                           rhs.inv_plugin_infos);

  return equal;
}
bool KinematicsPluginInfo::operator!=(const KinematicsPluginInfo& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******          ContactManagersPluginInfo           *****/
/*********************************************************/
void ContactManagersPluginInfo::insert(const ContactManagersPluginInfo& other)
{
  search_paths.insert(search_paths.end(), other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(search_libraries.end(), other.search_libraries.begin(), other.search_libraries.end());

  if (!other.discrete_plugin_infos.default_plugin.empty())
    discrete_plugin_infos.default_plugin = other.discrete_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.discrete_plugin_infos.plugins)
    discrete_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.continuous_plugin_infos.default_plugin.empty())
    continuous_plugin_infos.default_plugin = other.continuous_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.continuous_plugin_infos.plugins)
    continuous_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.discrete_plugin_infos.default_plugin.empty())
    discrete_plugin_infos.default_plugin = other.discrete_plugin_infos.default_plugin;

  if (!other.continuous_plugin_infos.default_plugin.empty())
    continuous_plugin_infos.default_plugin = other.continuous_plugin_infos.default_plugin;
}

void ContactManagersPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  discrete_plugin_infos.clear();
  continuous_plugin_infos.clear();
}

bool ContactManagersPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && discrete_plugin_infos.plugins.empty() &&
          continuous_plugin_infos.plugins.empty());
}

bool ContactManagersPluginInfo::operator==(const ContactManagersPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdentical<std::string>(search_paths, rhs.search_paths);
  equal &= isIdentical<std::string>(search_libraries, rhs.search_libraries);
  equal &= (discrete_plugin_infos == rhs.discrete_plugin_infos);
  equal &= (continuous_plugin_infos == rhs.continuous_plugin_infos);
  return equal;
}
bool ContactManagersPluginInfo::operator!=(const ContactManagersPluginInfo& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******          TaskComposerPluginInfo           *****/
/*********************************************************/
void TaskComposerPluginInfo::insert(const TaskComposerPluginInfo& other)
{
  search_paths.insert(search_paths.end(), other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(search_libraries.end(), other.search_libraries.begin(), other.search_libraries.end());

  if (!other.executor_plugin_infos.default_plugin.empty())
    executor_plugin_infos.default_plugin = other.executor_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.executor_plugin_infos.plugins)
    executor_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.task_plugin_infos.default_plugin.empty())
    task_plugin_infos.default_plugin = other.task_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.task_plugin_infos.plugins)
    task_plugin_infos.plugins[plugin_info.first] = plugin_info.second;
}

void TaskComposerPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  executor_plugin_infos.clear();
  task_plugin_infos.clear();
}

bool TaskComposerPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && executor_plugin_infos.plugins.empty() &&
          task_plugin_infos.plugins.empty());
}

bool TaskComposerPluginInfo::operator==(const TaskComposerPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdentical<std::string>(search_paths, rhs.search_paths);
  equal &= isIdentical<std::string>(search_libraries, rhs.search_libraries);
  equal &= (executor_plugin_infos == rhs.executor_plugin_infos);
  equal &= (task_plugin_infos == rhs.task_plugin_infos);
  return equal;
}
bool TaskComposerPluginInfo::operator!=(const TaskComposerPluginInfo& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_common
