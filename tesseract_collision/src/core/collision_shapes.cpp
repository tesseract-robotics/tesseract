/**
 * @file collision_shapes.cpp
 * @brief Tesseracts Collision Shapes Implementation
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

#include <tesseract_collision/core/collision_shapes.h>

namespace tesseract_collision
{
  // Base Class for Tesseract Collision Shapes

//  {
    CollisionShape::CollisionShape(CollisionShapeType type) : type_(type) {}
    CollisionShapeType CollisionShape::getType() const { return type_; }

    bool CollisionShape::isIdentical(const CollisionShape& shape1, const CollisionShape& shape2)
    {
      if (shape1.getType() != shape2.getType())
        return false;

      switch (shape1.getType())
      {
        case CollisionShapeType::BOX:
        {
          const BoxCollisionShape& s1 = static_cast<const BoxCollisionShape&>(shape1);
          const BoxCollisionShape& s2 = static_cast<const BoxCollisionShape&>(shape2);

          if (std::abs(s1.getX() - s2.getX()) > std::numeric_limits<double>::epsilon())
            return false;

          if (std::abs(s1.getY() - s2.getY()) > std::numeric_limits<double>::epsilon())
            return false;

          if (std::abs(s1.getZ() - s2.getZ()) > std::numeric_limits<double>::epsilon())
            return false;

          break;
        }
        case CollisionShapeType::SPHERE:
        {
          const SphereCollisionShape& s1 = static_cast<const SphereCollisionShape&>(shape1);
          const SphereCollisionShape& s2 = static_cast<const SphereCollisionShape&>(shape2);

          if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
            return false;

          break;
        }
        case CollisionShapeType::CYLINDER:
        {
          const CylinderCollisionShape& s1 = static_cast<const CylinderCollisionShape&>(shape1);
          const CylinderCollisionShape& s2 = static_cast<const CylinderCollisionShape&>(shape2);

          if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
            return false;

          if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
            return false;

          break;
        }
        case CollisionShapeType::CONE:
        {
          const ConeCollisionShape& s1 = static_cast<const ConeCollisionShape&>(shape1);
          const ConeCollisionShape& s2 = static_cast<const ConeCollisionShape&>(shape2);

          if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
            return false;

          if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
            return false;

          break;
        }
        case CollisionShapeType::MESH:
        {
          const MeshCollisionShape& s1 = static_cast<const MeshCollisionShape&>(shape1);
          const MeshCollisionShape& s2 = static_cast<const MeshCollisionShape&>(shape2);

          if (s1.getVerticeCount() != s2.getVerticeCount())
            return false;

          if (s1.getTriangleCount() != s2.getTriangleCount())
            return false;

          break;
        }
        case CollisionShapeType::CONVEX_MESH:
        {
          const ConvexMeshCollisionShape& s1 = static_cast<const ConvexMeshCollisionShape&>(shape1);
          const ConvexMeshCollisionShape& s2 = static_cast<const ConvexMeshCollisionShape&>(shape2);

          if (s1.getVerticeCount() != s2.getVerticeCount())
            return false;

          if (s1.getFaceCount() != s2.getFaceCount())
            return false;

          break;
        }
        case CollisionShapeType::SDF_MESH:
        {
          const SDFMeshCollisionShape& s1 = static_cast<const SDFMeshCollisionShape&>(shape1);
          const SDFMeshCollisionShape& s2 = static_cast<const SDFMeshCollisionShape&>(shape2);

          if (s1.getVerticeCount() != s2.getVerticeCount())
            return false;

          if (s1.getTriangleCount() != s2.getTriangleCount())
            return false;

          break;
        }
        case CollisionShapeType::OCTREE:
        {
          const OctreeCollisionShape& s1 = static_cast<const OctreeCollisionShape&>(shape1);
          const OctreeCollisionShape& s2 = static_cast<const OctreeCollisionShape&>(shape2);

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
        default:
        {
//          ROS_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(shape1.getType()));
          return false;
        }
      }

      return true;
    }
//  }

  // Box Collision Shape Implementation
//  {
    BoxCollisionShape::BoxCollisionShape(double x, double y, double z) : CollisionShape(CollisionShapeType::BOX), x_(x), y_(y), z_(z) {}
    double BoxCollisionShape::getX() const { return x_; }
    double BoxCollisionShape::getY() const { return y_; }
    double BoxCollisionShape::getZ() const { return z_; }
    CollisionShapePtr BoxCollisionShape::clone() const { return BoxCollisionShapePtr(new BoxCollisionShape(x_, y_, z_)); }
//  }

  // Sphere Collision Shape Implementation
//  {
    SphereCollisionShape::SphereCollisionShape(double r) : CollisionShape(CollisionShapeType::SPHERE), r_(r) {}
    double SphereCollisionShape::getRadius() const { return r_; }
    CollisionShapePtr SphereCollisionShape::clone() const { return SphereCollisionShapePtr(new SphereCollisionShape(r_)); }
//  }

  // Cylinder Collision Shape Implementation
//  {
    CylinderCollisionShape::CylinderCollisionShape(double r, double l) : CollisionShape(CollisionShapeType::CYLINDER), r_(r), l_(l) {}
    double CylinderCollisionShape::getRadius() const { return r_; }
    double CylinderCollisionShape::getLength() const { return l_; }
    CollisionShapePtr CylinderCollisionShape::clone() const { return CylinderCollisionShapePtr(new CylinderCollisionShape(r_, l_)); }
//  }

  // Cone Collision Shape Implementation
//  {
    ConeCollisionShape::ConeCollisionShape(double r, double l) : CollisionShape(CollisionShapeType::CONE), r_(r), l_(l) {}
    double ConeCollisionShape::getRadius() const { return r_; }
    double ConeCollisionShape::getLength() const { return l_; }
    CollisionShapePtr ConeCollisionShape::clone() const { return ConeCollisionShapePtr(new ConeCollisionShape(r_, l_)); }
//  }

  // Plane Collision Shape Implementation
//  {
    PlaneCollisionShape::PlaneCollisionShape(double a, double b, double c, double d) : CollisionShape(CollisionShapeType::PLANE), a_(a), b_(b), c_(c), d_(d) {}
    double PlaneCollisionShape::getA() const { return a_; }
    double PlaneCollisionShape::getB() const { return b_; }
    double PlaneCollisionShape::getC() const { return c_; }
    double PlaneCollisionShape::getD() const { return d_; }
    CollisionShapePtr PlaneCollisionShape::clone() const { return PlaneCollisionShapePtr(new PlaneCollisionShape(a_, b_, c_, d_)); }
//  }

  // Mesh Collision Shape Implementation
//  {
    MeshCollisionShape::MeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles) : CollisionShape(CollisionShapeType::MESH), vertices_(vertices), triangles_(triangles)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      triangle_count_ = 0;
      for (auto it = triangles_->begin(); it != triangles_->end(); ++it)
      {
        ++triangle_count_;
        int num_verts = *it;
        it = it + num_verts;
      }
    }

    MeshCollisionShape::MeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles, int triangle_count) : CollisionShape(CollisionShapeType::MESH), vertices_(vertices), triangles_(triangles), triangle_count_(triangle_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
    }

    const std::shared_ptr<const VectorVector3d>& MeshCollisionShape::getVertices() const { return vertices_; }
    const std::shared_ptr<const std::vector<int>>& MeshCollisionShape::getTriangles() const { return triangles_; }

    int MeshCollisionShape::getVerticeCount() const { return vertice_count_; }
    int MeshCollisionShape::getTriangleCount() const { return triangle_count_; }

    CollisionShapePtr MeshCollisionShape::clone() const { return MeshCollisionShapePtr(new MeshCollisionShape(vertices_, triangles_)); }
//  }

  // Convex Mesh Collision Shape Implementation
//  {
    ConvexMeshCollisionShape::ConvexMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces) : CollisionShape(CollisionShapeType::CONVEX_MESH), vertices_(vertices), faces_(faces)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      face_count_ = 0;
      for (auto it = faces_->begin(); it != faces_->end(); ++it)
      {
        ++face_count_;
        int num_verts = *it;
        it = it + num_verts;
      }
    }

    ConvexMeshCollisionShape::ConvexMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces, int face_count) : CollisionShape(CollisionShapeType::CONVEX_MESH), vertices_(vertices), faces_(faces), face_count_(face_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
    }

    const std::shared_ptr<const VectorVector3d>& ConvexMeshCollisionShape::getVertices() const { return vertices_; }
    const std::shared_ptr<const std::vector<int>>& ConvexMeshCollisionShape::getFaces() const { return faces_; }

    int ConvexMeshCollisionShape::getVerticeCount() const { return vertice_count_; }
    int ConvexMeshCollisionShape::getFaceCount() const { return face_count_; }

    CollisionShapePtr ConvexMeshCollisionShape::clone() const { return ConvexMeshCollisionShapePtr(new ConvexMeshCollisionShape(vertices_, faces_)); }
//  }

  // Octree Collision Shape Implementation
//  {
    OctreeCollisionShape::OctreeCollisionShape(const std::shared_ptr<const octomap::OcTree>& octree, SubShapeType sub_shape_type) : CollisionShape(CollisionShapeType::OCTREE), octree_(octree), sub_shape_type_(sub_shape_type) {}
    const std::shared_ptr<const octomap::OcTree>& OctreeCollisionShape::getOctree() const { return octree_; }
    OctreeCollisionShape::SubShapeType OctreeCollisionShape::getSubShapeType() const { return sub_shape_type_; }

    CollisionShapePtr OctreeCollisionShape::clone() const { return OctreeCollisionShapePtr(new OctreeCollisionShape(octree_, sub_shape_type_)); }
//  }

  // SDF Collision Shape Implementation
//  {
    SDFMeshCollisionShape::SDFMeshCollisionShape(const std::shared_ptr<const VectorVector3d> &vertices, const std::shared_ptr<const std::vector<int>>& triangles) : CollisionShape(CollisionShapeType::SDF_MESH), vertices_(vertices), triangles_(triangles)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      triangle_count_ = 0;
      for (auto it = triangles_->begin(); it != triangles_->end(); ++it)
      {
        ++triangle_count_;
        int num_verts = *it;
        it = it + num_verts;
      }
    }

    SDFMeshCollisionShape::SDFMeshCollisionShape(const std::shared_ptr<const VectorVector3d> &vertices, const std::shared_ptr<const std::vector<int>>& triangles, int triangle_count) : CollisionShape(CollisionShapeType::SDF_MESH), vertices_(vertices), triangles_(triangles), triangle_count_(triangle_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
    }
    const std::shared_ptr<const VectorVector3d>& SDFMeshCollisionShape::getVertices() const { return vertices_; }
    const std::shared_ptr<const std::vector<int>>& SDFMeshCollisionShape::getTriangles() const { return triangles_; }

    int SDFMeshCollisionShape::getVerticeCount() const { return vertice_count_; }
    int SDFMeshCollisionShape::getTriangleCount() const { return triangle_count_; }

    CollisionShapePtr SDFMeshCollisionShape::clone() const { return SDFMeshCollisionShapePtr(new SDFMeshCollisionShape(vertices_, triangles_)); }
//  }
}
