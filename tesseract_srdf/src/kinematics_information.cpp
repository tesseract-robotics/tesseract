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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/cereal_eigen_types.h>
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

  for (const auto& group : other.chain_groups)
    chain_groups[group.first] = group.second;

  for (const auto& group : other.joint_groups)
    joint_groups[group.first] = group.second;

  for (const auto& group : other.link_groups)
    link_groups[group.first] = group.second;

  for (const auto& group : other.group_states)
  {
    for (const auto& state : group.second)
      group_states[group.first][state.first] = state.second;
  }

  for (const auto& group : other.group_tcps)
  {
    for (const auto& tcp : group.second)
      group_tcps[group.first][tcp.first] = tcp.second;
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

bool KinematicsInformation::operator==(const KinematicsInformation& rhs) const
{
  auto double_eq = [](const double& v1, const double& v2) {
    return tesseract_common::almostEqualRelativeAndAbs(v1, v2, 1e-6, std::numeric_limits<float>::epsilon());
  };

  auto state_eq = [double_eq](const GroupsJointState& v1, const GroupsJointState& v2) {
    return tesseract_common::isIdenticalMap<GroupsJointState, double>(v1, v2, double_eq);
  };

  auto tcp_eq = [](const Eigen::Isometry3d& v1, const Eigen::Isometry3d& v2) { return v1.isApprox(v2, 1e-5); };

  auto list_eq = [](const std::vector<std::string>& v1, const std::vector<std::string>& v2) {
    return tesseract_common::isIdentical<std::string>(v1, v2, false);
  };

  bool equal = true;
  equal &= tesseract_common::isIdenticalSet<std::string>(group_names, rhs.group_names);
  equal &= tesseract_common::isIdenticalMap<ChainGroups, ChainGroup>(chain_groups, rhs.chain_groups);
  equal &= tesseract_common::isIdenticalMap<JointGroups, JointGroup>(joint_groups, rhs.joint_groups, list_eq);
  equal &= tesseract_common::isIdenticalMap<LinkGroups, LinkGroup>(link_groups, rhs.link_groups, list_eq);
  equal &= (kinematics_plugin_info == rhs.kinematics_plugin_info);

  equal &= (group_states.size() == rhs.group_states.size());
  if (equal)
  {
    for (const auto& group_state : group_states)
    {
      auto it_group = rhs.group_states.find(group_state.first);

      equal &= (it_group != rhs.group_states.end());
      if (!equal)
        break;

      equal &= tesseract_common::isIdenticalMap<GroupsJointStates, GroupsJointState>(
          group_state.second, it_group->second, state_eq);
      if (!equal)
        break;
    }
  }

  equal &= (group_tcps.size() == rhs.group_tcps.size());
  if (equal)
  {
    for (const auto& group_tcp : group_tcps)
    {
      auto it_group = rhs.group_tcps.find(group_tcp.first);

      equal &= (it_group != rhs.group_tcps.end());
      if (!equal)
        break;

      equal &=
          tesseract_common::isIdenticalMap<GroupsTCPs, Eigen::Isometry3d>(group_tcp.second, it_group->second, tcp_eq);
      if (!equal)
        break;
    }
  }

  return equal;
}
bool KinematicsInformation::operator!=(const KinematicsInformation& rhs) const { return !operator==(rhs); }

template <class Archive>
void KinematicsInformation::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(group_names);
  ar& BOOST_SERIALIZATION_NVP(chain_groups);
  ar& BOOST_SERIALIZATION_NVP(joint_groups);
  ar& BOOST_SERIALIZATION_NVP(link_groups);
  ar& BOOST_SERIALIZATION_NVP(group_states);
  ar& BOOST_SERIALIZATION_NVP(group_tcps);
  ar& BOOST_SERIALIZATION_NVP(kinematics_plugin_info);
}
}  // namespace tesseract_srdf

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_srdf::KinematicsInformation)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_srdf::KinematicsInformation)
