/**
 * @file kinematics_information.cpp
 * @brief This hold the kinematics information
 *
 * @author Levi Armstrong
 * @date May 12, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_srdf/kinematics_information.h>

namespace tesseract_srdf
{
void KinematicsInformation::clear()
{
  group_names.clear();
  chain_groups.clear();
  joint_groups.clear();
  link_groups.clear();
  group_states.clear();
  group_tcps.clear();
  kinematics_plugin_info.clear();
}

void KinematicsInformation::insert(const KinematicsInformation& other)
{
  group_names.insert(other.group_names.begin(), other.group_names.end());
  chain_groups.insert(other.chain_groups.begin(), other.chain_groups.end());
  joint_groups.insert(other.joint_groups.begin(), other.joint_groups.end());
  link_groups.insert(other.link_groups.begin(), other.link_groups.end());
  for (const auto& group : other.group_states)
  {
    auto it = group_states.find(group.first);
    if (it == group_states.end())
    {
      group_states[group.first] = group.second;
    }
    else
    {
      it->second.insert(group.second.begin(), group.second.end());
    }
  }

  for (const auto& group : other.group_tcps)
  {
    auto it = group_tcps.find(group.first);
    if (it == group_tcps.end())
    {
      group_tcps[group.first] = group.second;
    }
    else
    {
      it->second.insert(group.second.begin(), group.second.end());
    }
  }

  kinematics_plugin_info.insert(other.kinematics_plugin_info);
}

}  // namespace tesseract_srdf
