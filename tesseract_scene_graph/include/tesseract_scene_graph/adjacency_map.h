/**
 * @file adjacency_map.h
 * @brief Used to map links to its closes link in a provided list.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_SCENE_GRAPH_ADJACENCY_MAP_H
#define TESSERACT_SCENE_GRAPH_ADJACENCY_MAP_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/types.h>

#ifdef SWIG
%shared_ptr(tesseract_scene_graph::AdjacencyMapPair)
%shared_ptr(tesseract_scene_graph::AdjacencyMap)
#endif  // SWIG

namespace tesseract_scene_graph
{
/** @brief The AdjacencyMapPair struct */
struct AdjacencyMapPair
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<AdjacencyMapPair>;
  using ConstPtr = std::shared_ptr<const AdjacencyMapPair>;
  using UPtr = std::unique_ptr<AdjacencyMapPair>;
  using ConstUPtr = std::unique_ptr<const AdjacencyMapPair>;

  /** @brief The kinematic link associated with the adjacent link */
  std::string link_name;

  /** @brief A transform from the kinematic link (link_name) to the adjacent link */
  Eigen::Isometry3d transform;
};

class AdjacencyMap
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<AdjacencyMap>;
  using ConstPtr = std::shared_ptr<const AdjacencyMap>;
  using UPtr = std::unique_ptr<AdjacencyMap>;
  using ConstUPtr = std::unique_ptr<const AdjacencyMap>;

  /**
   * @brief Create a adjacency map provided state(map_links) and nearst parent in the active_links.
   *
   *        If a map_link does not have a parent in the list of active links it is not added the map
   *        Note: This currently only support tree structures.
   *        TODO: Need to update to use graph->getLinkChildren
   *
   * @param scene_graph
   * @param joint_link_names
   * @param state
   */
  AdjacencyMap(const SceneGraph& scene_graph,
               const std::vector<std::string>& joint_link_names,
               const tesseract_common::TransformMap& state);

  virtual ~AdjacencyMap() = default;
  AdjacencyMap(const AdjacencyMap& other);
  AdjacencyMap& operator=(const AdjacencyMap& other);
  AdjacencyMap(AdjacencyMap&&) = default;
  AdjacencyMap& operator=(AdjacencyMap&&) = default;

  /**
   * @brief This is a list of all active links associated with the constructor data.
   * @return vector of link names
   */
  const std::vector<std::string>& getActiveLinkNames() const;

  /**
   * @brief A link mapping to the associated kinematics link name if it exists
   * @param link_name Name of link
   * @return If the link does not have a associated kinematics link it return nullptr, otherwise return the pair.
   */
  const AdjacencyMapPair& getLinkMapping(const std::string& link_name) const;

private:
  std::vector<std::string> active_link_names_;
  std::unordered_map<std::string, AdjacencyMapPair::UPtr> adjacency_map_;
};
}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_ADJACENCY_MAP_H
