/**
 * @file utils.cpp
 * @brief Tesseract SRDF utility functions
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract/srdf/utils.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/srdf/srdf_model.h>

namespace tesseract::srdf
{
void processSRDFAllowedCollisions(tesseract::scene_graph::SceneGraph& scene_graph, const SRDFModel& srdf_model)
{
  for (const auto& [key, reason] : srdf_model.acm.getAllAllowedCollisions())
    scene_graph.addAllowedCollision(key, reason);
}

bool compareLinkPairAlphabetically(std::reference_wrapper<const tesseract::common::LinkIdPair> pair1,
                                   std::reference_wrapper<const tesseract::common::LinkIdPair> pair2)
{
  // orderedNameView() returns references into each pair, so the comparison copies no strings. Both
  // pairs outlive it.
  return pair1.get().orderedNameView() < pair2.get().orderedNameView();
}

std::vector<std::reference_wrapper<const tesseract::common::LinkIdPair>>
getAlphabeticalACMKeys(const tesseract::common::AllowedCollisionEntries& allowed_collision_entries)
{
  std::vector<std::reference_wrapper<const tesseract::common::LinkIdPair>> acm_keys;
  acm_keys.reserve(allowed_collision_entries.size());
  for (const auto& acm_pair : allowed_collision_entries)
  {
    acm_keys.push_back(std::ref(acm_pair.first));
  }

  // Sort the keys alphabetically
  sort(acm_keys.begin(), acm_keys.end(), compareLinkPairAlphabetically);

  return acm_keys;
}

}  // namespace tesseract::srdf
