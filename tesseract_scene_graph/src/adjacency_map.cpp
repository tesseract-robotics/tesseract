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

#include <tesseract_scene_graph/adjacency_map.h>

namespace tesseract_scene_graph
{
AdjacencyMap::AdjacencyMap(const tesseract_scene_graph::SceneGraph& scene_graph,
                           const std::vector<std::string>& joint_link_names,
                           const tesseract_common::TransformMap& state)
{
  assert(scene_graph.isTree());

  for (const auto& ml : state)
  {
    if (std::find(joint_link_names.begin(), joint_link_names.end(), ml.first) != joint_link_names.end())
    {
      auto pair = std::make_unique<AdjacencyMapPair>();
      pair->link_name = ml.first;
      pair->transform.setIdentity();
      adjacency_map_[ml.first] = std::move(pair);
      active_link_names_.push_back(ml.first);
      continue;
    }

    std::vector<std::string> inv_adj_links = scene_graph.getInvAdjacentLinkNames(ml.first);
    while (!inv_adj_links.empty())
    {
      assert(inv_adj_links.size() == 1);

      const std::string& ial = inv_adj_links[0];
      auto it = std::find(joint_link_names.begin(), joint_link_names.end(), ial);
      if (it != joint_link_names.end())
      {
        auto pair = std::make_unique<AdjacencyMapPair>();
        pair->link_name = ial;
        pair->transform = state.at(ial).inverse() * ml.second;
        adjacency_map_[ml.first] = std::move(pair);
        active_link_names_.push_back(ml.first);
        break;
      }

      inv_adj_links = scene_graph.getInvAdjacentLinkNames(ial);
    }
  }
}

AdjacencyMap::AdjacencyMap(const AdjacencyMap& other)
{
  active_link_names_ = other.active_link_names_;
  for (const auto& pair : adjacency_map_)
    adjacency_map_[pair.first] = std::make_unique<AdjacencyMapPair>(*pair.second);
}

AdjacencyMap& AdjacencyMap::operator=(const AdjacencyMap& other)
{
  active_link_names_ = other.active_link_names_;
  for (const auto& pair : adjacency_map_)
    adjacency_map_[pair.first] = std::make_unique<AdjacencyMapPair>(*pair.second);

  return *this;
}

/**
 * @brief This is a list of all active links associated with the constructor data.
 * @return vector of link names
 */
const std::vector<std::string>& AdjacencyMap::getActiveLinkNames() const { return active_link_names_; }

/**
 * @brief A link mapping to the associated kinematics link name if it exists
 * @param link_name Name of link
 * @return If the link does not have a associated kinematics link it return nullptr, otherwise return the pair.
 */
const AdjacencyMapPair& AdjacencyMap::getLinkMapping(const std::string& link_name) const
{
  //  const auto& it = adjacency_map_.find(link_name);
  //  if (it == adjacency_map_.end())
  //    return nullptr;

  //  return *(it->second);
  return *(adjacency_map_.at(link_name));
}
}  // namespace tesseract_scene_graph
