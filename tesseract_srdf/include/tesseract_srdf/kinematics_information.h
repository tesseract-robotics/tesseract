/**
 * @file kinematics_information.h
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
#ifndef TESSERACT_SRDF_KINEMATICS_INFORMATION_H
#define TESSERACT_SRDF_KINEMATICS_INFORMATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_srdf
{
using GroupsJointState = std::unordered_map<std::string, double>;
using GroupsJointStates = std::unordered_map<std::string, GroupsJointState>;
using GroupJointStates = std::unordered_map<std::string, GroupsJointStates>;
using GroupsTCPs = tesseract_common::AlignedMap<std::string, Eigen::Isometry3d>;
using GroupTCPs = tesseract_common::AlignedMap<std::string, GroupsTCPs>;
using ChainGroup = std::vector<std::pair<std::string, std::string>>;
using ChainGroups = std::unordered_map<std::string, ChainGroup>;
using JointGroup = std::vector<std::string>;
using JointGroups = std::unordered_map<std::string, JointGroup>;
using LinkGroup = std::vector<std::string>;
using LinkGroups = std::unordered_map<std::string, LinkGroup>;
using GroupNames = std::set<std::string>;

/**
 * @brief This hold the kinematics information used to create the SRDF and is the data
 * container for the manipulator manager.
 */
struct KinematicsInformation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief A set of group names */
  GroupNames group_names;

  /** @brief A map of chains groups*/
  ChainGroups chain_groups;

  /** @brief A map of joint groups */
  JointGroups joint_groups;

  /** @brief A map of link groups */
  LinkGroups link_groups;

  /** @brief A map of group states */
  GroupJointStates group_states;

  /** @brief A map of group tool center points */
  GroupTCPs group_tcps;

  /** @brief The kinematics plugin information */
  tesseract_common::KinematicsPluginInfo kinematics_plugin_info;

  /** @brief Insert the content of an other KinematicsInformation */
  void insert(const KinematicsInformation& other);

  /** @brief Clear the kinematics information */
  void clear();

  /** @brief Check if group exists */
  bool hasGroup(const std::string& group_name) const;

  /** @brief Add chain group */
  void addChainGroup(const std::string& group_name, const ChainGroup& chain_group);

  /** @brief Remove chain group */
  void removeChainGroup(const std::string& group_name);

  /** @brief Check if chain group exists */
  bool hasChainGroup(const std::string& group_name) const;

  /** @brief Add joint group */
  void addJointGroup(const std::string& group_name, const JointGroup& joint_group);

  /** @brief Remove joint group */
  void removeJointGroup(const std::string& group_name);

  /** @brief Check if joint group exists */
  bool hasJointGroup(const std::string& group_name) const;

  /** @brief Add link group */
  void addLinkGroup(const std::string& group_name, const LinkGroup& link_group);

  /** @brief Remove link group */
  void removeLinkGroup(const std::string& group_name);

  /** @brief Check if link group exists */
  bool hasLinkGroup(const std::string& group_name) const;

  /** @brief Add group joint state */
  void addGroupJointState(const std::string& group_name,
                          const std::string& state_name,
                          const GroupsJointState& joint_state);

  /** @brief Remove group joint state */
  void removeGroupJointState(const std::string& group_name, const std::string& state_name);

  /** @brief Check if group joint state exists */
  bool hasGroupJointState(const std::string& group_name, const std::string& state_name) const;

  /** @brief Add group tool center point */
  void addGroupTCP(const std::string& group_name, const std::string& tcp_name, const Eigen::Isometry3d& tcp);

  /** @brief Remove group tool center point */
  void removeGroupTCP(const std::string& group_name, const std::string& tcp_name);

  /** @brief Check if group tool center point exists */
  bool hasGroupTCP(const std::string& group_name, const std::string& tcp_name) const;
};

}  // namespace tesseract_srdf
#endif  // TESSERACT_SRDF_KINEMATICS_INFORMATION_H
