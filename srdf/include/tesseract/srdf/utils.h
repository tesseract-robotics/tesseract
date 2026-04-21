/**
 * @file utils.h
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
#ifndef TESSERACT_SRDF_UTILS_H
#define TESSERACT_SRDF_UTILS_H

#include <string>

#include <tesseract/common/types.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::srdf
{
class SRDFModel;

/**
 * @brief True iff @p name resolves to a link registered in @p scene_graph with exactly that name.
 *
 * Returns false in two cases, indistinguishable by design:
 *   1. The name is not registered at all, or
 *   2. The name hashes to the same LinkId as an already-registered link with a different stored
 *      name, a hash collision.
 *
 * Both are equally disqualifying for SRDF name resolution, so callers need not distinguish.
 */
bool isRegisteredLink(const tesseract::scene_graph::SceneGraph& scene_graph, const std::string& name);

/**
 * @brief True iff @p name resolves to a joint registered in @p scene_graph with exactly that name.
 *
 * Returns false in two cases, indistinguishable by design:
 *   1. The name is not registered at all, or
 *   2. The name hashes to the same JointId as an already-registered joint with a different stored
 *      name, a hash collision.
 *
 * Both are equally disqualifying for SRDF name resolution, so callers need not distinguish.
 */
bool isRegisteredJoint(const tesseract::scene_graph::SceneGraph& scene_graph, const std::string& name);

/**
 * @brief Add allowed collisions to the scene graph
 * @param scene_graph The scene graph to add allowed collisions data
 * @param srdf_model The srdf model to extract allowed collisions
 */
void processSRDFAllowedCollisions(tesseract::scene_graph::SceneGraph& scene_graph, const SRDFModel& srdf_model);

/**
 * @brief Returns a deterministically alphabetically sorted vector of ACM entries.
 *
 * Entries stored in an AllowedCollisionMatrix have `name1`/`name2` ordered by LinkId hash value
 * (see common::orderedPairNames), not alphabetically — and std::hash<std::string> is not portable
 * across standard library implementations, so that order is unstable for cross-library output.
 * This function normalizes each entry so `name1 <= name2` alphabetically, then sorts the entries
 * by (name1, name2), producing output suitable for deterministic serialization.
 *
 * @param allowed_collision_entries Entries to be sorted
 * @return An alphabetically sorted vector of ACM entries (by name1, then name2), with each entry's
 *         names themselves in alphabetical order.
 */
std::vector<tesseract::common::ACMEntry>
getAlphabeticalACMEntries(const tesseract::common::AllowedCollisionEntries& allowed_collision_entries);
}  // namespace tesseract::srdf
#endif  // TESSERACT_SRDF_UTILS_H
