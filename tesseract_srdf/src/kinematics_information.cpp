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

bool KinematicsInformation::hasGroup(const std::string& group_name) const
{
  return std::find(group_names.begin(), group_names.end(), group_name) != group_names.end();
}

void KinematicsInformation::addChainGroup(const std::string& group_name, const ChainGroup& chain_group)
{
  chain_groups[group_name] = chain_group;
  group_names.insert(group_name);
}

void KinematicsInformation::removeChainGroup(const std::string& group_name)
{
  if (chain_groups.erase(group_name) > 0)
    group_names.erase(group_name);
}

bool KinematicsInformation::hasChainGroup(const std::string& group_name) const
{
  return (chain_groups.find(group_name) != chain_groups.end());
}

void KinematicsInformation::addJointGroup(const std::string& group_name, const JointGroup& joint_group)
{
  joint_groups[group_name] = joint_group;
  group_names.insert(group_name);
}

void KinematicsInformation::removeJointGroup(const std::string& group_name)
{
  if (joint_groups.erase(group_name) > 0)
    group_names.erase(group_name);
}

bool KinematicsInformation::hasJointGroup(const std::string& group_name) const
{
  return (joint_groups.find(group_name) != joint_groups.end());
}

void KinematicsInformation::addLinkGroup(const std::string& group_name, const LinkGroup& link_group)
{
  link_groups[group_name] = link_group;
  group_names.insert(group_name);
}

void KinematicsInformation::removeLinkGroup(const std::string& group_name)
{
  if (link_groups.erase(group_name) > 0)
    group_names.erase(group_name);
}

bool KinematicsInformation::hasLinkGroup(const std::string& group_name) const
{
  return (link_groups.find(group_name) != link_groups.end());
}

void KinematicsInformation::addGroupJointState(const std::string& group_name,
                                               const std::string& state_name,
                                               const GroupsJointState& joint_state)
{
  group_states[group_name][state_name] = joint_state;
}

void KinematicsInformation::removeGroupJointState(const std::string& group_name, const std::string& state_name)
{
  group_states[group_name].erase(state_name);

  if (group_states[group_name].empty())
    group_states.erase(group_name);
}

bool KinematicsInformation::hasGroupJointState(const std::string& group_name, const std::string& state_name) const
{
  auto it = group_states.find(group_name);
  if (it == group_states.end())
    return false;

  return (it->second.find(state_name) != it->second.end());
}

void KinematicsInformation::addGroupTCP(const std::string& group_name,
                                        const std::string& tcp_name,
                                        const Eigen::Isometry3d& tcp)
{
  group_tcps[group_name][tcp_name] = tcp;
}

void KinematicsInformation::removeGroupTCP(const std::string& group_name, const std::string& tcp_name)
{
  group_tcps.at(group_name).erase(tcp_name);

  if (group_tcps[group_name].empty())
    group_tcps.erase(group_name);
}

bool KinematicsInformation::hasGroupTCP(const std::string& group_name, const std::string& tcp_name) const
{
  auto it = group_tcps.find(group_name);
  if (it == group_tcps.end())
    return false;

  return (it->second.find(tcp_name) != it->second.end());
}
}  // namespace tesseract_srdf
