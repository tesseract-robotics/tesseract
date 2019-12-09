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
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
btCollisionShape* createShapePrimitive(const tesseract_geometry::Box::ConstPtr& geom)
{
  auto a = static_cast<btScalar>(geom->getX() / 2);
  auto b = static_cast<btScalar>(geom->getY() / 2);
  auto c = static_cast<btScalar>(geom->getZ() / 2);

  return (new btBoxShape(btVector3(a, b, c)));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return (new btSphereShape(static_cast<btScalar>(geom->getRadius())));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength() / 2);
  return (new btCylinderShapeZ(btVector3(r, r, l)));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Cone::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength());
  return (new btConeShapeZ(r, l));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Capsule::ConstPtr& geom)
{
  auto r = static_cast<btScalar>(geom->getRadius());
  auto l = static_cast<btScalar>(geom->getLength());
  return (new btCapsuleShapeZ(r, l));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Mesh::ConstPtr& geom,
                                       CollisionObjectWrapper* cow,
                                       int shape_index)
{
  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getTriangleCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());
  const Eigen::VectorXi& triangles = *(geom->getTriangles());

  if (vertice_count > 0 && triangle_count > 0)
  {
    auto* compound = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(triangle_count));
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative

    for (int i = 0; i < triangle_count; ++i)
    {
      btVector3 v[3];
      assert(triangles[4 * i] == 3);
      for (unsigned x = 0; x < 3; ++x)
      {
        // Note: triangles structure is number of vertices that represent the triangle followed by vertex indexes
        const Eigen::Vector3d& vertice = vertices[static_cast<size_t>(triangles[(4 * i) + (static_cast<int>(x) + 1)])];
        for (unsigned y = 0; y < 3; ++y)
          v[x][y] = static_cast<btScalar>(vertice[y]);
      }

      btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
      if (subshape != nullptr)
      {
        subshape->setUserIndex(shape_index);
        cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geomTrans;
        geomTrans.setIdentity();
        compound->addChildShape(geomTrans, subshape);
      }
    }

    return compound;
  }
  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getFaceCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());

  if (vertice_count > 0 && triangle_count > 0)
  {
    auto* subshape = new btConvexHullShape();
    for (const auto& v : vertices)
      subshape->addPoint(
          btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

    return subshape;
  }
  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::Octree::ConstPtr& geom,
                                       CollisionObjectWrapper* cow,
                                       int shape_index)
{
  const octomap::OcTree& octree = *(geom->getOctree());
  auto* subshape = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(octree.size()));
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
          auto l = static_cast<btScalar>(size / 2);
          auto* childshape = new btBoxShape(btVector3(l, l, l));
          childshape->setUserIndex(shape_index);
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
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
          auto* childshape = new btSphereShape(static_cast<btScalar>((size / 2)));
          childshape->setUserIndex(shape_index);
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
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
          auto* childshape = new btSphereShape(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
          childshape->setUserIndex(shape_index);
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
        }
      }
      return subshape;
    }
  }

  CONSOLE_BRIDGE_logError("This bullet shape type (%d) is not supported for geometry octree",
                          static_cast<int>(geom->getSubType()));
  return nullptr;
}

btCollisionShape* createShapePrimitive(const CollisionShapeConstPtr& geom, CollisionObjectWrapper* cow, int shape_index)
{
  btCollisionShape* shape = nullptr;

  switch (geom->getType())
  {
    case tesseract_geometry::GeometryType::BOX:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Box>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Mesh>(geom), cow, shape_index);
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
      shape->setUserIndex(shape_index);
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      shape = createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Octree>(geom), cow, shape_index);
      shape->setUserIndex(shape_index);
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
    btCollisionShape* shape = createShapePrimitive(m_shapes[0], this, 0);
    shape->setMargin(BULLET_MARGIN);
    manage(shape);
    setCollisionShape(shape);
  }
  else
  {
    auto* compound = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
    manage(compound);
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative
    setCollisionShape(compound);

    for (std::size_t j = 0; j < m_shapes.size(); ++j)
    {
      btCollisionShape* subshape = createShapePrimitive(m_shapes[j], this, static_cast<int>(j));
      if (subshape != nullptr)
      {
        manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geomTrans = convertEigenToBt(m_shape_poses[j]);
        compound->addChildShape(geomTrans, subshape);
      }
    }
  }

  btTransform trans;
  trans.setIdentity();
  setWorldTransform(trans);
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract_common::VectorIsometry3d shape_poses,
                                               std::vector<std::shared_ptr<void>> data)
  : m_name(std::move(name))
  , m_type_id(type_id)
  , m_shapes(std::move(shapes))
  , m_shape_poses(std::move(shape_poses))
  , m_data(std::move(data))
{
}
}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
