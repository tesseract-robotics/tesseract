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
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_srdf
{
/** @brief A structure to hold opw kinematics data */
struct OPWKinematicParameters
{
  double a1 = 0, a2 = 0, b = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0;
  std::array<double, 6> offsets = { 0, 0, 0, 0, 0, 0 };
  std::array<signed char, 6> sign_corrections = { 1, 1, 1, 1, 1, 1 };

  bool operator==(const OPWKinematicParameters& rhs) const;
  bool operator!=(const OPWKinematicParameters& rhs) const;
};

struct ROPKinematicParameters
{
  /** @brief The name of the solver. If empty it will use the default solver name */
  std::string solver_name;
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;

  bool operator==(const ROPKinematicParameters& rhs) const;
  bool operator!=(const ROPKinematicParameters& rhs) const;
};

struct REPKinematicParameters
{
  /** @brief The name of the solver. If empty it will use the default solver name */
  std::string solver_name;
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;

  bool operator==(const REPKinematicParameters& rhs) const;
  bool operator!=(const REPKinematicParameters& rhs) const;
};

}  // namespace tesseract_srdf

#ifdef SWIG
%template(GroupOPWKinematics) std::unordered_map<std::string, tesseract_srdf::OPWKinematicParameters>;
%template(GroupROPKinematics) std::unordered_map<std::string, tesseract_srdf::ROPKinematicParameters>;
%template(GroupREPKinematics) std::unordered_map<std::string, tesseract_srdf::REPKinematicParameters>;
#endif  // SWIG

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
using GroupNames = std::vector<std::string>;
using GroupROPKinematics = std::unordered_map<std::string, ROPKinematicParameters>;
using GroupREPKinematics = std::unordered_map<std::string, REPKinematicParameters>;
using GroupOPWKinematics = std::unordered_map<std::string, OPWKinematicParameters>;
using GroupDefaultKinematicsSolver = std::unordered_map<std::string, std::string>;

/**
 * @brief This hold the kinematics information used to create the SRDF and is the data
 * container for the manipulator manager.
 */
struct KinematicsInformation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief A vector of group names */
  std::vector<std::string> group_names;

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

  /** @brief A map of group opw kinematics data */
  GroupOPWKinematics group_opw_kinematics;

  /** @brief A map of robot on positioner groups */
  GroupROPKinematics group_rop_kinematics;

  /** @brief A map of robot with external positioner groups */
  GroupREPKinematics group_rep_kinematics;

  /** @brief A map of group default forward kinematics solvers */
  GroupDefaultKinematicsSolver group_default_fwd_kin;

  /** @brief A map of group default forward kinematics solvers */
  GroupDefaultKinematicsSolver group_default_inv_kin;

  /** @brief Clear the kinematics information */
  void clear();
};

}  // namespace tesseract_srdf
#endif  // TESSERACT_SRDF_KINEMATICS_INFORMATION_H
