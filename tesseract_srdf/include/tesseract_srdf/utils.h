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

}  // namespace tesseract_srdf
#endif  // TESSERACT_SRDF_UTILS_H
