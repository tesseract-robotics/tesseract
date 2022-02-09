/**
 * @file utils.h
 * @brief Tesseract SRDF utility functions
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_SRDF_UTILS_H
#define TESSERACT_SRDF_UTILS_H

#include <functional>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/types.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/srdf_model.h>

namespace tesseract_srdf
{
/**
 * @brief Add allowed collisions to the scene graph
 * @param scene_graph The scene graph to add allowed collisions data
 * @param srdf_model The srdf model to extract allowed collisions
 */
void processSRDFAllowedCollisions(tesseract_scene_graph::SceneGraph& scene_graph, const SRDFModel& srdf_model);

/**
 * @brief Used to sort a pair of strings alphabetically - first by the pair.first and then by pair.second
 * @param pair1 First pair of strings
 * @param pair2 Second pair of strings
 * @return True if pair1 should go before pair2 (is closer to A)
 */
bool compareLinkPairAlphabetically(std::reference_wrapper<const tesseract_common::LinkNamesPair> pair1,
                                   std::reference_wrapper<const tesseract_common::LinkNamesPair> pair2);

/**
 * @brief Returns an alphabetically sorted vector of ACM keys (the link pairs)
 * @param allowed_collision_entries Entries to be sorted
 * @return An alphabetically sorted vector of ACM keys (the link pairs)
 */
std::vector<std::reference_wrapper<const tesseract_common::LinkNamesPair>>
getAlphabeticalACMKeys(const tesseract_common::AllowedCollisionEntries& allowed_collision_entries);
}  // namespace tesseract_srdf
#endif  // TESSERACT_SRDF_UTILS_H
