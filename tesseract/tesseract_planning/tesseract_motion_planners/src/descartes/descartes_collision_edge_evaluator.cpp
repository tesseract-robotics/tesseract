/**
 * @file descartes_collision_edge_evaluator.cpp
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#include <tesseract_motion_planners/descartes/impl/descartes_collision_edge_evaluator.hpp>

namespace tesseract_motion_planners
{
// Explicit template instantiation
template class DescartesCollisionEdgeEvaluator<float>;
template class DescartesCollisionEdgeEvaluator<double>;

}  // namespace tesseract_motion_planners
