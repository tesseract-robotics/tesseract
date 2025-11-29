/**
 * @file utils.h
 * @brief Tesseract Geometry Utility Function
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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
#ifndef TESSERACT_GEOMETRY_UTILS_H
#define TESSERACT_GEOMETRY_UTILS_H

#include <Eigen/Geometry>
#include <tesseract_common/eigen_types.h>

namespace tesseract_geometry
{
class Geometry;

/**
 * @brief Check if two Geometries are identical
 * @param geom1 First Geometry
 * @param geom2 Second Geometry
 * @return True if identical, otherwise false
 */
bool isIdentical(const Geometry& geom1, const Geometry& geom2);

/**
 * @brief A utility for extracting vertices from a geomtry
 * If it is a primitive shape, it will be converted to a mesh and then extract vertices
 * @param geom The geometry to extract vertices from
 * @param origin The origin of the geometry transform the extracted vertices
 * @return The extracted vertices
 */
tesseract_common::VectorVector3d extractVertices(const Geometry& geom, const Eigen::Isometry3d& origin);

}  // namespace tesseract_geometry
#endif
