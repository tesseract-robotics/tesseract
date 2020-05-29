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

using JointState = std::unordered_map<std::string, double>;
using JointStates = std::unordered_map<std::string, JointState>;
using GroupStates = std::unordered_map<std::string, JointStates>;
using TCPs = std::unordered_map<std::string, Eigen::Isometry3d>;
using GroupTCPs = std::unordered_map<std::string, TCPs>;
using ChainGroups = std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>;
using JointGroups = std::unordered_map<std::string, std::vector<std::string>>;
using LinkGroups = std::unordered_map<std::string, std::vector<std::string>>;
using GroupNames = std::vector<std::string>;
using GroupOPWKinematics = std::unordered_map<std::string, OPWKinematicParameters>;

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_SRDF_TYPES_H
