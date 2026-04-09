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
#include <tesseract/srdf/srdf_model.h>

namespace tesseract::srdf
{
void processSRDFAllowedCollisions(tesseract::scene_graph::SceneGraph& scene_graph, const SRDFModel& srdf_model)
{
  for (const auto& [key, entry] : srdf_model.acm.getAllAllowedCollisions())
    scene_graph.addAllowedCollision(entry.name1, entry.name2, entry.reason);
}

std::vector<std::reference_wrapper<const tesseract::common::ACMEntry>>
getAlphabeticalACMEntries(const tesseract::common::AllowedCollisionEntries& allowed_collision_entries)
{
  std::vector<std::reference_wrapper<const tesseract::common::ACMEntry>> entries;
  entries.reserve(allowed_collision_entries.size());
  for (const auto& [key, entry] : allowed_collision_entries)
    entries.push_back(std::cref(entry));

  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
    if (a.get().name1 != b.get().name1)
      return a.get().name1 < b.get().name1;
    return a.get().name2 < b.get().name2;
  });

  return entries;
}

}  // namespace tesseract::srdf
