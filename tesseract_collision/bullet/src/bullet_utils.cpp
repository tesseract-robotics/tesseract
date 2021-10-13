/**
 * @file bullet_utils.cpp
 * @brief Tesseract ROS Bullet environment utility function.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "tesseract_collision/bullet/bullet_utils.h"

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <LinearMath/btConvexHullComputer.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btTriangleShapeEx.h>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{
int createConvexHull(tesseract_common::VectorVector3d& vertices,
                     Eigen::VectorXi& faces,
                     const tesseract_common::VectorVector3d& input,
                     double shrink,
                     double shrinkClamp)
{
  vertices.clear();

  btConvexHullComputer conv;
  std::vector<double> points;
  points.reserve(input.size() * 3);
  for (const auto& v : input)
  {
    points.push_back(v[0]);
    points.push_back(v[1]);
    points.push_back(v[2]);
  }

  btScalar val = conv.compute(points.data(),
                              3 * sizeof(double),
                              static_cast<int>(input.size()),
                              static_cast<btScalar>(shrink),
                              static_cast<btScalar>(shrinkClamp));
  if (val < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(static_cast<size_t>(num_verts));
  for (int i = 0; i < num_verts; i++)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(
        Eigen::Vector3d(static_cast<double>(v.getX()), static_cast<double>(v.getY()), static_cast<double>(v.getZ())));
  }

  auto num_faces = static_cast<size_t>(conv.faces.size());
  std::vector<int> local_faces;
  local_faces.reserve(3UL * num_faces);
  for (int i = 0; i < conv.faces.size(); i++)
  {
    std::vector<int> face;
    face.reserve(3);

    const btConvexHullComputer::Edge* sourceEdge = &(conv.edges[conv.faces[i]]);
    int a = sourceEdge->getSourceVertex();
    face.push_back(a);

    int b = sourceEdge->getTargetVertex();
    face.push_back(b);

    const btConvexHullComputer::Edge* edge = sourceEdge->getNextEdgeOfFace();
    int c = edge->getTargetVertex();
    face.push_back(c);

    edge = edge->getNextEdgeOfFace();
    c = edge->getTargetVertex();
    while (c != a)
    {
      face.push_back(c);

      edge = edge->getNextEdgeOfFace();
      c = edge->getTargetVertex();
    }
    local_faces.push_back(static_cast<int>(face.size()));
    local_faces.insert(local_faces.end(), face.begin(), face.end());
  }

  faces.resize(static_cast<long>(local_faces.size()));
  for (size_t i = 0; i < local_faces.size(); ++i)
    faces[static_cast<long>(i)] = local_faces[i];

  return conv.faces.size();
}

namespace tesseract_collision_bullet
{
std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Box::ConstPtr& geom)
{
  auto a = static_cast<btScalar>(geom->getX() / 2);
  auto b = static_cast<btScalar>(geom->getY() / 2);
  auto c = static_cast<btScalar>(geom->getZ() / 2);

  return std::make_shared<btBoxShape>(btVector3(a, b, c));
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return std::make_shared<btSphereShape>(static_cast<btScalar>(geom->getRadius()));
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength() / 2);
  return std::make_shared<btCylinderShapeZ>(btVector3(r, r, l));
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Cone::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength());
  return std::make_shared<btConeShapeZ>(r, l);
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Capsule::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength());
  return std::make_shared<btCapsuleShapeZ>(r, l);
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Mesh::ConstPtr& geom,
                                                       CollisionObjectWrapper* cow,
                                                       int shape_index)
{
  int vertice_count = geom->getVertexCount();
  int triangle_count = geom->getFaceCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());
  const Eigen::VectorXi& triangles = *(geom->getFaces());

  if (vertice_count > 0 && triangle_count > 0)
  {
    auto compound =
        std::make_shared<btCompoundShape>(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(triangle_count));
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative

    for (int i = 0; i < triangle_count; ++i)
    {
      btVector3 v[3];  // NOLINT
      assert(triangles[4 * i] == 3);
      for (unsigned x = 0; x < 3; ++x)
      {
        // Note: triangles structure is number of vertices that represent the triangle followed by vertex indexes
        const Eigen::Vector3d& vertice = vertices[static_cast<size_t>(triangles[(4 * i) + (static_cast<int>(x) + 1)])];
        for (unsigned y = 0; y < 3; ++y)
          v[x][y] = static_cast<btScalar>(vertice[y]);
      }

      std::shared_ptr<btCollisionShape> subshape = std::make_shared<btTriangleShapeEx>(v[0], v[1], v[2]);
      if (subshape != nullptr)
      {
        subshape->setUserIndex(shape_index);
        cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geomTrans;
        geomTrans.setIdentity();
        compound->addChildShape(geomTrans, subshape.get());
      }
    }

    return compound;
  }
  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  int vertice_count = geom->getVertexCount();
  int triangle_count = geom->getFaceCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());

  if (vertice_count > 0 && triangle_count > 0)
  {
    auto subshape = std::make_shared<btConvexHullShape>();
    for (const auto& v : vertices)
      subshape->addPoint(
          btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

    return subshape;
  }
  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const tesseract_geometry::Octree::ConstPtr& geom,
                                                       CollisionObjectWrapper* cow,
                                                       int shape_index)
{
  const octomap::OcTree& octree = *(geom->getOctree());
  auto subshape = std::make_shared<btCompoundShape>(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(octree.size()));
  double occupancy_threshold = octree.getOccupancyThres();

  switch (geom->getSubType())
  {
    case tesseract_geometry::Octree::SubType::BOX:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          auto l = static_cast<btScalar>(size / 2.0);
          auto childshape = std::make_shared<btBoxShape>(btVector3(l, l, l));
          childshape->setUserIndex(shape_index);
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }
      return subshape;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_INSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          auto childshape = std::make_shared<btSphereShape>(static_cast<btScalar>((size / 2)));
          childshape->setUserIndex(shape_index);
          // Sphere is a special case where you do not modify the margin which is internally set to the radius
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }
      return subshape;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          auto childshape =
              std::make_shared<btSphereShape>(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
          childshape->setUserIndex(shape_index);
          // Sphere is a special case where you do not modify the margin which is internally set to the radius
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }
      return subshape;
    }
  }

  CONSOLE_BRIDGE_logError("This bullet shape type (%d) is not supported for geometry octree",
                          static_cast<int>(geom->getSubType()));
  return nullptr;
}

std::shared_ptr<btCollisionShape> createShapePrimitive(const CollisionShapeConstPtr& geom,
                                                       CollisionObjectWrapper* cow,
                                                       int shape_index)
{
  std::shared_ptr<btCollisionShape> shape = nullptr;

  switch (geom->getType())
  {
    case tesseract_geometry::GeometryType::BOX:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Box>(geom));
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
      shape->setUserIndex(shape_index);
      // Sphere is a special case where you do not modify the margin which is internally set to the radius
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Mesh>(geom), cow, shape_index);
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Octree>(geom), cow, shape_index);
      shape->setUserIndex(shape_index);
      shape->setMargin(BULLET_MARGIN);
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using BULLET yet",
                              static_cast<int>(geom->getType()));
      break;
    }
  }

  return shape;
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract_common::VectorIsometry3d shape_poses)
  : m_name(std::move(name)), m_type_id(type_id), m_shapes(std::move(shapes)), m_shape_poses(std::move(shape_poses))
{
  assert(!m_shapes.empty());
  assert(!m_shape_poses.empty());
  assert(!m_name.empty());
  assert(m_shapes.size() == m_shape_poses.size());

  m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
  m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;

  if (m_shapes.size() == 1 && m_shape_poses[0].matrix().isIdentity())
  {
    std::shared_ptr<btCollisionShape> shape = createShapePrimitive(m_shapes[0], this, 0);
    manage(shape);
    setCollisionShape(shape.get());
  }
  else
  {
    auto compound =
        std::make_shared<btCompoundShape>(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
    manage(compound);
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative
    setCollisionShape(compound.get());

    for (std::size_t j = 0; j < m_shapes.size(); ++j)
    {
      std::shared_ptr<btCollisionShape> subshape = createShapePrimitive(m_shapes[j], this, static_cast<int>(j));
      if (subshape != nullptr)
      {
        manage(subshape);
        btTransform geomTrans = convertEigenToBt(m_shape_poses[j]);
        compound->addChildShape(geomTrans, subshape.get());
      }
    }
  }

  btTransform trans;
  trans.setIdentity();
  setWorldTransform(trans);
}

}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
