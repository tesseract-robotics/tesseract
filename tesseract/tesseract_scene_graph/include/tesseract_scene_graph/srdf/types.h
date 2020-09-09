#ifndef TESSERACT_SCENE_GRAPH_SRDF_TYPES_H
#define TESSERACT_SCENE_GRAPH_SRDF_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_scene_graph
{
/** @brief A structure to hold opw kinematics data */
struct OPWKinematicParameters
{
  double a1{ 0 }, a2{ 0 }, b{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, c4{ 0 };
  double offsets[6]{ 0, 0, 0, 0, 0, 0 };
  signed char sign_corrections[6]{ 1, 1, 1, 1, 1, 1 };
};

struct ROPKinematicParameters
{
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;
};

struct REPKinematicParameters
{
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;
};

using GroupsJointState = std::unordered_map<std::string, double>;
using GroupsJointStates = std::unordered_map<std::string, GroupsJointState>;
using GroupJointStates = std::unordered_map<std::string, GroupsJointStates>;
using GroupsTCPs = std::unordered_map<std::string, Eigen::Isometry3d>;
using GroupTCPs = std::unordered_map<std::string, GroupsTCPs>;
using ChainGroup = std::vector<std::pair<std::string, std::string>>;
using ChainGroups = std::unordered_map<std::string, ChainGroup>;
using JointGroup = std::vector<std::string>;
using JointGroups = std::unordered_map<std::string, JointGroup>;
using LinkGroup = std::vector<std::string>;
using LinkGroups = std::unordered_map<std::string, LinkGroup>;
using GroupNames = std::vector<std::string>;
using ROPGroups = std::unordered_map<std::string, ROPKinematicParameters>;
using REPGroups = std::unordered_map<std::string, REPKinematicParameters>;
using GroupOPWKinematics = std::unordered_map<std::string, OPWKinematicParameters>;

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_SRDF_TYPES_H
