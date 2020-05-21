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
#include <tesseract_scene_graph/srdf_model.h>
%}

%shared_ptr(tesseract_scene_graph::SRDFModel)

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

class SRDFModel
{
public:
  using Ptr = std::shared_ptr<SRDFModel>;
  using ConstPtr = std::shared_ptr<const SRDFModel>;

  using JointState = std::unordered_map<std::string, double>;
  using JointStates = std::unordered_map<std::string, JointState>;
  using GroupStates = std::unordered_map<std::string, JointStates>;
  using TCPs = std::unordered_map<std::string, Eigen::Isometry3d>;
  using GroupTCPs = std::unordered_map<std::string, TCPs>;
  using ChainGroups = std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>;
  using JointGroups = std::unordered_map<std::string, std::vector<std::string>>;
  using LinkGroups = std::unordered_map<std::string, std::vector<std::string>>;
  using GroupOPWKinematics = std::unordered_map<std::string, OPWKinematicParameters>;

  /** @brief Load Model from TiXMLElement */
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  /** @brief Load Model from TiXMLDocument */
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLDocument* srdf_xml);

  /** @brief Load Model given a filename */
  bool initFile(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& filename);

  /** @brief Load Model from a XML-string */
  bool initString(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& xmlstring);

  /** @brief Save the model to a file */
  bool saveToFile(const std::string& file_path) const;

  /** @brief Get the name of this model */
  const std::string getName() const;
  std::string getName();

  /** @brief Get the list of pairs of links that need not be checked for collisions (because they can never touch given the geometry and kinematics of the robot) */
  const AllowedCollisionMatrix& getAllowedCollisionMatrix() const;
  AllowedCollisionMatrix& getAllowedCollisionMatrix();

  /** @brief Get the map of chain groups defined for this model */
  const ChainGroups& getChainGroups() const;
  ChainGroups& getChainGroups();

  /** @brief Get the map of joint groups defined for this model */
  const JointGroups& getJointGroups() const;
  JointGroups& getJointGroups();

  /** @brief Get the map of groups defined for this model */
  const LinkGroups& getLinkGroups() const;
  LinkGroups& getLinkGroups();

  /** @brief Get the map of group tool center points (TCP) defined for this model */
  const GroupTCPs& getGroupTCPs() const;
  GroupTCPs& getGroupTCPs();

  /** @brief Get the map of group states defined for this model */
  const GroupStates& getGroupStates() const;
  GroupStates& getGroupStates();

  /** @brief Get the map of group opw kinematics defined for this model */
  const GroupOPWKinematics& getGroupOPWKinematics() const;
  GroupOPWKinematics& getGroupOPWKinematics();

  /** @brief Clear the model */
  void clear();
};

}  // namespace tesseract_scene_graph

