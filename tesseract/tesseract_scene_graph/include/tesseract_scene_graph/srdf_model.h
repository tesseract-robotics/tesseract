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
#include <fstream>
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kinematics_information.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>

/// Main namespace
namespace tesseract_scene_graph
{
/** @brief Representation of semantic information about the robot */
class SRDFModel
{
public:
  using Ptr = std::shared_ptr<SRDFModel>;
  using ConstPtr = std::shared_ptr<const SRDFModel>;

  SRDFModel() = default;
  virtual ~SRDFModel() = default;
  SRDFModel(const SRDFModel&) = default;
  SRDFModel& operator=(const SRDFModel&) = default;
  SRDFModel(SRDFModel&&) = default;
  SRDFModel& operator=(SRDFModel&&) = default;

  /** @brief Load Model from TiXMLElement */
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, const tinyxml2::XMLElement* srdf_xml);

  /** @brief Load Model from TiXMLDocument */
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, const tinyxml2::XMLDocument* srdf_xml);

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

  /** @brief Get the kinematics information */
  const KinematicsInformation& getKinematicsInformation() const;
  KinematicsInformation& getKinematicsInformation();

  /** @brief Clear the model */
  void clear();

private:
  std::string name_{ "undefined" };              /**< @brief The name of the srdf model */
  std::array<int, 3> version_{ { 1, 0, 0 } };    /**< @brief The version number major.minor[.patch] */
  AllowedCollisionMatrix acm_;                   /**< @brief The allowed collision matrix */
  KinematicsInformation kinematics_information_; /**< @brief A map of group opw kinematics data */
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_MODEL_H
