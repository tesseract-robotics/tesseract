/**
 * @file types.h
 * @brief The tesseract_environment package types.
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
#ifndef TESSERACT_ENVIRONMENT_TYPES_H
#define TESSERACT_ENVIRONMENT_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>
#include <map>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
/** @brief This holds a state of the environment */
struct EnvState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<EnvState>;
  using ConstPtr = std::shared_ptr<const EnvState>;

  std::unordered_map<std::string, double> joints;
  tesseract_common::TransformMap transforms;
};

/**
 * @brief The AdjacencyMapPair struct
 */
struct AdjacencyMapPair
{
  using Ptr = std::shared_ptr<AdjacencyMapPair>;
  using ConstPtr = std::shared_ptr<const AdjacencyMapPair>;

  std::string link_name;       /**< @brief The kinematic link associated with the adjacent link */
  Eigen::Isometry3d transform; /**< @brief A transform from the kinematic link (link_name) to the adjacent link */
};

class AdjacencyMap
{
public:
  using Ptr = std::shared_ptr<AdjacencyMap>;
  using ConstPtr = std::shared_ptr<const AdjacencyMap>;

  /**
   * @brief Create a adjacency map provided state(map_links) and nearst parent in the active_links.
   *
   *        If a map_link does not have a parent in the list of active links it is not added the map
   *        Note: This currently only support tree structures.
   *        TODO: Need to update to use graph->getLinkChildren
   *
   * @param scene_graph
   * @param active_links
   * @param state
   */
  AdjacencyMap(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
               const std::vector<std::string>& active_links,
               const tesseract_common::TransformMap& state)
  {
    assert(scene_graph->isTree());

    for (const auto& ml : state)
    {
      if (std::find(active_links.begin(), active_links.end(), ml.first) != active_links.end())
      {
        AdjacencyMapPair::Ptr pair = std::make_shared<AdjacencyMapPair>();
        pair->link_name = ml.first;
        pair->transform.setIdentity();
        adjacency_map_[ml.first] = pair;
        active_link_names_.push_back(ml.first);
        continue;
      }

      std::vector<std::string> inv_adj_links = scene_graph->getInvAdjacentLinkNames(ml.first);
      while (!inv_adj_links.empty())
      {
        assert(inv_adj_links.size() == 1);

        const std::string& ial = inv_adj_links[0];
        auto it = std::find(active_links.begin(), active_links.end(), ial);
        if (it != active_links.end())
        {
          AdjacencyMapPair::Ptr pair = std::make_shared<AdjacencyMapPair>();
          pair->link_name = ial;
          pair->transform = state.at(ial).inverse() * ml.second;
          adjacency_map_[ml.first] = pair;
          active_link_names_.push_back(ml.first);
          break;
        }

        inv_adj_links = scene_graph->getInvAdjacentLinkNames(ial);
      }
    }
  }

  virtual ~AdjacencyMap() = default;
  AdjacencyMap(const AdjacencyMap&) = default;
  AdjacencyMap& operator=(const AdjacencyMap&) = default;
  AdjacencyMap(AdjacencyMap&&) = default;
  AdjacencyMap& operator=(AdjacencyMap&&) = default;

  /**
   * @brief This is a list of all active links associated with the constructor data.
   * @return vector of link names
   */
  const std::vector<std::string>& getActiveLinkNames() const { return active_link_names_; }

  /**
   * @brief A link mapping to the associated kinematics link name if it exists
   * @param link_name Name of link
   * @return If the link does not have a associated kinematics link it return nullptr, otherwise return the pair.
   */
  AdjacencyMapPair::ConstPtr getLinkMapping(const std::string& link_name) const
  {
    const auto& it = adjacency_map_.find(link_name);
    if (it == adjacency_map_.end())
      return nullptr;

    return it->second;
  }

private:
  std::vector<std::string> active_link_names_;
  std::unordered_map<std::string, AdjacencyMapPair::ConstPtr> adjacency_map_;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_TYPES_H
