/**
 * @file types.cpp
 * @brief Common Tesseract Types
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

#include <tesseract_common/types.h>

namespace tesseract_common
{
void PluginInfoContainer::clear()
{
  default_plugin.clear();
  plugins.clear();
}

void KinematicsPluginInfo::insert(const KinematicsPluginInfo& other)
{
  search_paths.insert(other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(other.search_libraries.begin(), other.search_libraries.end());
  fwd_plugin_infos.insert(other.fwd_plugin_infos.begin(), other.fwd_plugin_infos.end());
  inv_plugin_infos.insert(other.inv_plugin_infos.begin(), other.inv_plugin_infos.end());
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

void ContactManagersPluginInfo::insert(const ContactManagersPluginInfo& other)
{
  search_paths.insert(other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(other.search_libraries.begin(), other.search_libraries.end());
  discrete_plugin_infos.plugins.insert(other.discrete_plugin_infos.plugins.begin(),
                                       other.discrete_plugin_infos.plugins.end());
  continuous_plugin_infos.plugins.insert(other.continuous_plugin_infos.plugins.begin(),
                                         other.continuous_plugin_infos.plugins.end());

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

std::size_t PairHash::operator()(const LinkNamesPair& pair) const
{
  return std::hash<std::string>()(pair.first + pair.second);
}

LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
{
  if (link_name1 <= link_name2)
    return std::make_pair(link_name1, link_name2);

  return std::make_pair(link_name2, link_name1);
}

}  // namespace tesseract_common
