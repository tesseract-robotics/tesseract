/**
 * @file srdf_model.h
 * @brief Parse srdf xml
 *
 * @author Levi Armstrong, Ioan Sucan
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

#ifndef TESSERACT_SCENE_GRAPH_SRDF_MODEL_H
#define TESSERACT_SCENE_GRAPH_SRDF_MODEL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <tinyxml2.h>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <fstream>
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

/// Main namespace
namespace tesseract_scene_graph
{

/** @brief A structure to hold opw kinematics data */
struct OPWKinematicParameters
{
  double a1{ 0 }, a2{ 0 }, b{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, c4{ 0 };
  double offsets[6]{ 0, 0, 0, 0, 0, 0 };
  signed char sign_corrections[6]{ 1, 1, 1, 1, 1, 1 };
};

/** @brief Representation of semantic information about the robot */
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

  SRDFModel() = default;
  virtual ~SRDFModel() = default;
  SRDFModel(const SRDFModel&) = default;
  SRDFModel& operator=(const SRDFModel&) = default;
  SRDFModel(SRDFModel&&) = default;
  SRDFModel& operator=(SRDFModel&&) = default;

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
  const std::string& getName() const;
  std::string& getName();

  /** @brief Get the version number {major, minor, patch} */
  const std::array<int, 3>& getVersion() const;

  /** @brief Get the allowed collision matrix */
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

private:
  /**
   * @brief Load groups from srdf xml element
   * @param scene_graph The tesseract scene graph
   * @param srdf_xml The xml element to parse
   */
  void loadGroups(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  /**
   * @brief Load groups states from srdf xml element
   * @param scene_graph The tesseract scene graph
   * @param srdf_xml The xml element to parse
   */
  void loadGroupStates(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  /**
   * @brief Load groups tool center points from srdf xml element
   * @param scene_graph The tesseract scene graph
   * @param srdf_xml The xml element to parse
   */
  void loadToolCenterPoints(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  /**
   * @brief Load allowed collisions from srdf xml element
   * @param scene_graph The tesseract scene graph
   * @param srdf_xml The xml element to parse
   */
  void loadDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  /**
   * @brief Load group opw kinematics from srdf xml element
   * @param scene_graph The tesseract scene graph
   * @param srdf_xml The xml element to parse
   */
  void loadGroupOPWKinematics(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml);

  std::string name_{ "undefined" };         /**< @brief The name of the srdf model */
  std::array<int, 3> version_{ 1, 0, 0 };   /**< @brief The version number major.minor[.patch] */
  ChainGroups chain_groups_;                /**< @brief A map of chains groups*/
  JointGroups joint_groups_;                /**< @brief A map of joint groups */
  LinkGroups link_groups_;                  /**< @brief A map of link groups */
  std::vector<std::string> group_names_;    /**< @brief A vector of group names */
  GroupStates group_states_;                /**< @brief  A map of group states*/
  GroupTCPs group_tcps_;                    /**< @brief A map of group tool center points */
  AllowedCollisionMatrix acm_;              /**< @brief The allowed collision matrix */
  GroupOPWKinematics group_opw_kinematics_; /**< @brief A map of group opw kinematics data */
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_MODEL_H
