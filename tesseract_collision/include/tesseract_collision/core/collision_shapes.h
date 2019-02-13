/**
 * @file collision_shapes.h
 * @brief Tesseracts Collision Shapes
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
#ifndef TESSERACT_COLLISION_COLLISION_SHAPES_H
#define TESSERACT_COLLISION_COLLISION_SHAPES_H

#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{
  enum CollisionShapeType
  {
    SPHERE,
    CYLINDER,
    CONE,
    BOX,
    PLANE,
    MESH,
    CONVEX_MESH,
    SDF_MESH,
    OCTREE
  };

  class CollisionShape;
  typedef std::shared_ptr<CollisionShape> CollisionShapePtr;
  typedef std::shared_ptr<const CollisionShape> CollisionShapeConstPtr;
  typedef std::vector<CollisionShapePtr> CollisionShapes;
  typedef std::vector<CollisionShapeConstPtr> CollisionShapesConst;

  class CollisionShape
  {
  public:
    explicit CollisionShape(CollisionShapeType type);
    virtual ~CollisionShape() = default;

    /** \brief Create a copy of this shape */
    virtual CollisionShapePtr clone() const = 0;

    CollisionShapeType getType() const;

    static bool isIdentical(const CollisionShape& shape1, const CollisionShape& shape2);

  private:
    /** \brief The type of the shape */
    CollisionShapeType type_;
  };

  class BoxCollisionShape : public CollisionShape
  {
  public:
    BoxCollisionShape(double x, double y, double z);
    ~BoxCollisionShape() override = default;

    double getX() const;
    double getY() const;
    double getZ() const;

    CollisionShapePtr clone() const override;

  private:
    double x_;
    double y_;
    double z_;
  };
  typedef std::shared_ptr<BoxCollisionShape> BoxCollisionShapePtr;
  typedef std::shared_ptr<const BoxCollisionShape> BoxCollisionShapeConstPtr;

  class SphereCollisionShape : public CollisionShape
  {
  public:
    explicit SphereCollisionShape(double r);
    ~SphereCollisionShape() override = default;

    double getRadius() const;

    CollisionShapePtr clone() const override;

  private:
    double r_;
  };
  typedef std::shared_ptr<SphereCollisionShape> SphereCollisionShapePtr;
  typedef std::shared_ptr<const SphereCollisionShape> SphereCollisionShapeConstPtr;

  class CylinderCollisionShape : public CollisionShape
  {
  public:
    CylinderCollisionShape(double r, double l);
    ~CylinderCollisionShape() override = default;

    double getRadius() const;
    double getLength() const;

    CollisionShapePtr clone() const override;

  private:
    double r_;
    double l_;
  };
  typedef std::shared_ptr<CylinderCollisionShape> CylinderCollisionShapePtr;
  typedef std::shared_ptr<const CylinderCollisionShape> CylinderCollisionShapeConstPtr;

  class ConeCollisionShape : public CollisionShape
  {
  public:
    ConeCollisionShape(double r, double l);
    ~ConeCollisionShape() override = default;

    double getRadius() const;
    double getLength() const;

    CollisionShapePtr clone() const override;

  private:
    double r_;
    double l_;
  };
  typedef std::shared_ptr<ConeCollisionShape> ConeCollisionShapePtr;
  typedef std::shared_ptr<const ConeCollisionShape> ConeCollisionShapeConstPtr;

  class PlaneCollisionShape : public CollisionShape
  {
  public:
    PlaneCollisionShape(double a, double b, double c, double d);
    ~PlaneCollisionShape() override = default;

    double getA() const;
    double getB() const;
    double getC() const;
    double getD() const;

    CollisionShapePtr clone() const override;

  private:
    double a_;
    double b_;
    double c_;
    double d_;
  };
  typedef std::shared_ptr<PlaneCollisionShape> PlaneCollisionShapePtr;
  typedef std::shared_ptr<const PlaneCollisionShape> PlaneCollisionShapeConstPtr;

  class MeshCollisionShape : public CollisionShape
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles);
    MeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles, int triangle_count);
    ~MeshCollisionShape() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const;
    const std::shared_ptr<const std::vector<int>>& getTriangles() const;

    int getVerticeCount() const;
    int getTriangleCount() const;

    CollisionShapePtr clone() const override;

  private:
    std::shared_ptr<const VectorVector3d> vertices_;
    std::shared_ptr<const std::vector<int>> triangles_;
    int vertice_count_;
    int triangle_count_;
  };
  typedef std::shared_ptr<MeshCollisionShape> MeshCollisionShapePtr;
  typedef std::shared_ptr<const MeshCollisionShape> MeshCollisionShapeConstPtr;

  class ConvexMeshCollisionShape : public CollisionShape
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConvexMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces);
    ConvexMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& faces, int face_count);
    ~ConvexMeshCollisionShape() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const;
    const std::shared_ptr<const std::vector<int>>& getFaces() const;

    int getVerticeCount() const;
    int getFaceCount() const;

    CollisionShapePtr clone() const override;

  private:
    std::shared_ptr<const VectorVector3d> vertices_;
    std::shared_ptr<const std::vector<int>> faces_;

    int vertice_count_;
    int face_count_;
  };
  typedef std::shared_ptr<ConvexMeshCollisionShape> ConvexMeshCollisionShapePtr;
  typedef std::shared_ptr<const ConvexMeshCollisionShape> ConvexMeshCollisionShapeConstPtr;

  class OctreeCollisionShape : public CollisionShape
  {
  public:
    enum SubShapeType
    {
      BOX,
      SPHERE_INSIDE,
      SPHERE_OUTSIDE
    };

    OctreeCollisionShape(const std::shared_ptr<const octomap::OcTree>& octree, SubShapeType sub_shape_type);
    ~OctreeCollisionShape() override = default;

    const std::shared_ptr<const octomap::OcTree>& getOctree() const;
    SubShapeType getSubShapeType() const;

    CollisionShapePtr clone() const override;

    /**
     * @brief Octrees are typically generated from 3D sensor data so this method
     * should be used to efficiently update the collision shape.
     */
    void update();

  private:
    std::shared_ptr<const octomap::OcTree> octree_;
    SubShapeType sub_shape_type_;
  };
  typedef std::shared_ptr<OctreeCollisionShape> OctreeCollisionShapePtr;
  typedef std::shared_ptr<const OctreeCollisionShape> OctreeCollisionShapeConstPtr;

  class SDFMeshCollisionShape : public CollisionShape
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SDFMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles);
    SDFMeshCollisionShape(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const std::vector<int>>& triangles, int triangle_count);
    ~SDFMeshCollisionShape() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const;
    const std::shared_ptr<const std::vector<int>>& getTriangles() const;

    int getVerticeCount() const;
    int getTriangleCount() const;

    CollisionShapePtr clone() const override;

    private:
      std::shared_ptr<const VectorVector3d> vertices_;
      std::shared_ptr<const std::vector<int>> triangles_;

      int vertice_count_;
      int triangle_count_;
  };
  typedef std::shared_ptr<SDFMeshCollisionShape> SDFMeshCollisionShapePtr;
  typedef std::shared_ptr<const SDFMeshCollisionShape> SDFMeshCollisionShapeConstPtr;

}
#endif // TESSERACT_COLLISION_COLLISION_SHAPES_H
