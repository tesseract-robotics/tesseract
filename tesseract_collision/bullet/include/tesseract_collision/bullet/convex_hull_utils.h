/**
 * @file convex_hull_utils.h
 * @brief This is a collection of common methods
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
#ifndef TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H
#define TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H

#include <tesseract_common/types.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>

namespace tesseract_collision
{
/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertices
 * index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occurred when trying to create the convex hull
 */
int createConvexHull(tesseract_common::VectorVector3d& vertices,
                     Eigen::VectorXi& faces,
                     const tesseract_common::VectorVector3d& input,
                     double shrink = -1,
                     double shrinkClamp = -1);

tesseract_geometry::ConvexMesh::Ptr makeConvexMesh(const tesseract_geometry::Mesh& mesh);

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_BULLET_CONVEX_HULL_UTILS_H
