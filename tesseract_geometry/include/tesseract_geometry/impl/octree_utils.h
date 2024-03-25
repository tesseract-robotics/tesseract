/**
 * @file octree.h
 * @brief Tesseract Octree Geometry Utils
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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
#ifndef TESSERACT_GEOMETRY_OCTREE_UTILS_H
#define TESSERACT_GEOMETRY_OCTREE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/OcTree.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>

namespace tesseract_geometry
{
template <typename PointT>
std::unique_ptr<octomap::OcTree>
createOctree(const PointT& point_cloud, const double resolution, const bool prune, const bool binary = true)
{
  auto ot = std::make_unique<octomap::OcTree>(resolution);

  for (auto& point : point_cloud.points)
    ot->updateNode(point.x, point.y, point.z, true, true);

  // Per the documentation for overload updateNode above with lazy_eval enabled this must be called after all points
  // are added
  ot->updateInnerOccupancy();
  if (binary)
    ot->toMaxLikelihood();

  if (prune)
    tesseract_geometry::Octree::prune(*ot);

  return ot;
}
}  // namespace tesseract_geometry
#endif  // TESSERACT_GEOMETRY_OCTREE_UTILS_H
