/**
 * @file tesseract_geometry_fwd.h
 * @brief Tesseract Geometry Forward Declarations
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
#ifndef TESSERACT_GEOMETRY_TESSERACT_GEOMETRY_FWD_H
#define TESSERACT_GEOMETRY_TESSERACT_GEOMETRY_FWD_H

#include <cstdint>

namespace tesseract_geometry
{
enum class GeometryType : std::uint8_t;
class Geometry;
class Box;
class Capsule;
class Cone;
class ConvexMesh;
class Cylinder;
class MeshMaterial;
class MeshTexture;
class Mesh;
class Octree;
enum class OctreeSubType : std::uint8_t;
class Plane;
class PolygonMesh;
class SDFMesh;
class Sphere;
}  // namespace tesseract_geometry
#endif  // TESSERACT_GEOMETRY_TESSERACT_GEOMETRY_FWD_H
