/**
 * @file srdf_parser.i
 * @brief SWIG interface file for tesseract_scene_graph/parser/srdf_parser.h
 *
 * @author Hervé Audren
 * @date January 20, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Ascent Robotics Inc.
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

%{
#include <tesseract_scene_graph/srdf/types.h>
%}

%template(map_string_isometry3d) std::unordered_map<std::string, Eigen::Isometry3d>;

%template(map_string_opwkinparams) std::unordered_map<std::string, tesseract_scene_graph::OPWKinematicParameters>;

namespace tesseract_scene_graph
{

/** @brief A structure to hold opw kinematics data */
struct OPWKinematicParameters
{
  double a1, a2, b, c1, c2, c3, c4;
  double offsets[6];
  signed char sign_corrections[6];
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

}
