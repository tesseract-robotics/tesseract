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
  for (const auto& [key, entry] : srdf_model.acm.getAllAllowedCollisions())
    scene_graph.addAllowedCollision(key, entry);
}

std::vector<AlphabeticalACMEntry>
getAlphabeticalACMEntries(const tesseract::common::AllowedCollisionEntries& allowed_collision_entries)
{
  std::vector<AlphabeticalACMEntry> entries;
  entries.reserve(allowed_collision_entries.size());
  for (const auto& [key, entry] : allowed_collision_entries)
  {
    std::string name1 = key.first().name();
    std::string name2 = key.second().name();
    if (name2 < name1)
      std::swap(name1, name2);
    entries.push_back(AlphabeticalACMEntry{ std::move(name1), std::move(name2), entry.reason });
  }

  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
    if (a.name1 != b.name1)
      return a.name1 < b.name1;
    return a.name2 < b.name2;
  });

  return entries;
}

}  // namespace tesseract::srdf
