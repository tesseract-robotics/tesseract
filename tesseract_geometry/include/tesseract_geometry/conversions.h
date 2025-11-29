/**
 * @file conversions.h
 * @brief Tesseract Geometry Conversion Functions
 *
 * @author Levi Armstrong
 * @date July 27, 2025
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#ifndef TESSERACT_GEOMETRY_CONVERSIONS_H
#define TESSERACT_GEOMETRY_CONVERSIONS_H

#include <memory>
#include <Eigen/Geometry>

namespace tesseract_geometry
{
class Geometry;
class Mesh;

/**
 * @brief Convert geometry type to mesh
 * @details Currently this only converts primitive geometry types to mesh
 * @param geom The geometry to convert
 * @param tolerance The allowed chord deviation
 * @return The mesh representation of the geometry type provided
 */
std::unique_ptr<Mesh> toTriangleMesh(const Geometry& geom,
                                     double tolerance,
                                     const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

}  // namespace tesseract_geometry

#endif  // TESSERACT_GEOMETRY_CONVERSIONS_H
