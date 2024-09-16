/**
 * @file utils.cpp
 * @brief Tesseract Geometry Utility Function
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/utils.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_geometry
{
bool isIdentical(const Geometry& geom1, const Geometry& geom2)
{
  if (geom1.getType() != geom2.getType())
    return false;

  switch (geom1.getType())
  {
    case GeometryType::BOX:
    {
      const auto& s1 = static_cast<const Box&>(geom1);
      const auto& s2 = static_cast<const Box&>(geom2);

      if (std::abs(s1.getX() - s2.getX()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getY() - s2.getY()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getZ() - s2.getZ()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::SPHERE:
    {
      const auto& s1 = static_cast<const Sphere&>(geom1);
      const auto& s2 = static_cast<const Sphere&>(geom2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::CYLINDER:
    {
      const auto& s1 = static_cast<const Cylinder&>(geom1);
      const auto& s2 = static_cast<const Cylinder&>(geom2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::CONE:
    {
      const auto& s1 = static_cast<const Cone&>(geom1);
      const auto& s2 = static_cast<const Cone&>(geom2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::CAPSULE:
    {
      const auto& s1 = static_cast<const Capsule&>(geom1);
      const auto& s2 = static_cast<const Capsule&>(geom2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::PLANE:
    {
      const auto& s1 = static_cast<const Plane&>(geom1);
      const auto& s2 = static_cast<const Plane&>(geom2);

      if (std::abs(s1.getA() - s2.getA()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getB() - s2.getB()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getC() - s2.getC()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getD() - s2.getD()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case GeometryType::MESH:
    {
      const auto& s1 = static_cast<const Mesh&>(geom1);
      const auto& s2 = static_cast<const Mesh&>(geom2);

      if (s1.getVertexCount() != s2.getVertexCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case GeometryType::CONVEX_MESH:
    {
      const auto& s1 = static_cast<const ConvexMesh&>(geom1);
      const auto& s2 = static_cast<const ConvexMesh&>(geom2);

      if (s1.getVertexCount() != s2.getVertexCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case GeometryType::SDF_MESH:
    {
      const auto& s1 = static_cast<const SDFMesh&>(geom1);
      const auto& s2 = static_cast<const SDFMesh&>(geom2);

      if (s1.getVertexCount() != s2.getVertexCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case GeometryType::OCTREE:
    {
      const auto& s1 = static_cast<const Octree&>(geom1);
      const auto& s2 = static_cast<const Octree&>(geom2);

      const std::shared_ptr<const octomap::OcTree>& octree1 = s1.getOctree();
      const std::shared_ptr<const octomap::OcTree>& octree2 = s2.getOctree();

      if (octree1->getTreeType() != octree2->getTreeType())
        return false;

      if (octree1->size() != octree2->size())
        return false;

      if (octree1->getTreeDepth() != octree2->getTreeDepth())
        return false;

      if (octree1->memoryUsage() != octree2->memoryUsage())
        return false;

      if (octree1->memoryFullGrid() != octree2->memoryFullGrid())
        return false;

      break;
    }
    case GeometryType::POLYGON_MESH:
    {
      const auto& s1 = static_cast<const PolygonMesh&>(geom1);
      const auto& s2 = static_cast<const PolygonMesh&>(geom2);

      if (s1.getVertexCount() != s2.getVertexCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case GeometryType::COMPOUND_MESH:
    {
      const auto& s1 = static_cast<const CompoundMesh&>(geom1);
      const auto& s2 = static_cast<const CompoundMesh&>(geom2);

      if (s1.getMeshes().size() != s2.getMeshes().size())
        return false;

      for (std::size_t i = 0; i < s1.getMeshes().size(); ++i)
      {
        if (!isIdentical(*s1.getMeshes()[i], *s2.getMeshes()[i]))
          return false;
      }

      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported", static_cast<int>(geom1.getType()));
      return false;
    }
  }

  return true;
}
}  // namespace tesseract_geometry
