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

TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
btCollisionShape* createShapePrimitive(const tesseract_geometry::BoxConstPtr& geom)
{
  btScalar a = static_cast<btScalar>(geom->getX() / 2);
  btScalar b = static_cast<btScalar>(geom->getY() / 2);
  btScalar c = static_cast<btScalar>(geom->getZ() / 2);

  return (new btBoxShape(btVector3(a, b, c)));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::SphereConstPtr& geom)
{
  return (new btSphereShape(static_cast<btScalar>(geom->getRadius())));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::CylinderConstPtr& geom)
{
  btScalar r = static_cast<btScalar>(geom->getRadius());
  btScalar l = static_cast<btScalar>(geom->getLength() / 2);
  return (new btCylinderShapeZ(btVector3(r, r, l)));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::ConeConstPtr& geom)
{
  btScalar r = static_cast<btScalar>(geom->getRadius());
  btScalar l = static_cast<btScalar>(geom->getLength());
  return (new btConeShapeZ(r, l));
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::MeshConstPtr& geom,
                                       CollisionObjectWrapper* cow)
{

  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getTriangleCount();
  const VectorVector3d& vertices = *(geom->getVertices());
  const std::vector<int>& triangles = *(geom->getTriangles());

  if (vertice_count > 0 && triangle_count > 0)
  {
    btCompoundShape* compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(triangle_count));
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative

    for (int i = 0; i < triangle_count; ++i)
    {
      btVector3 v[3];
      assert(triangles[static_cast<size_t>(4 * i)] == 3);
      for (unsigned x = 0; x < 3; ++x)
      {
        // Note: triangles structure is number of vertices that represent the triangle followed by vertex indexes
        const Eigen::Vector3d& vertice = vertices[static_cast<size_t>(triangles[(4 * static_cast<size_t>(i)) + (x + 1)])];
        for (unsigned y = 0; y < 3; ++y)
          v[x][y] = static_cast<btScalar>(vertice[y]);
      }

      btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
      if (subshape != nullptr)
      {
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

btCollisionShape* createShapePrimitive(const tesseract_geometry::ConvexMeshConstPtr& geom)
{

  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getFaceCount();
  const VectorVector3d& vertices = *(geom->getVertices());

  if (vertice_count > 0 && triangle_count > 0)
  {
    btConvexHullShape* subshape = new btConvexHullShape();
    for (const auto& v : vertices)
      subshape->addPoint(
          btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

    return subshape;
  }
  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

btCollisionShape* createShapePrimitive(const tesseract_geometry::OctreeConstPtr& geom,
                                       CollisionObjectWrapper* cow)
{

  const octomap::OcTree& octree = *(geom->getOctree());
  btCompoundShape* subshape =
      new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(octree.size()));
  double occupancy_threshold = octree.getOccupancyThres();

  switch (geom->getSubType())
  {
    case tesseract_geometry::Octree::SubType::BOX:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())),
                end = octree.end();
           it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          btScalar l = static_cast<btScalar>(size / 2);
          btBoxShape* childshape = new btBoxShape(btVector3(l, l, l));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
        }
      }
      return subshape;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_INSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())),
                end = octree.end();
           it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          btSphereShape* childshape = new btSphereShape(static_cast<btScalar>((size / 2)));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
        }
      }
      return subshape;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end; ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geomTrans;
          geomTrans.setIdentity();
          geomTrans.setOrigin(btVector3(
              static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
          btSphereShape* childshape =
              new btSphereShape(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geomTrans, childshape);
        }
      }
      return subshape;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This bullet shape type (%d) is not supported for geometry octree",
                static_cast<int>(geom->getSubType()));
      return nullptr;
    }
  }
}

btCollisionShape* createShapePrimitive(const CollisionShapeConstPtr& geom,
                                       CollisionObjectWrapper* cow)
{
  switch (geom->getType())
  {
    case tesseract_geometry::GeometryType::BOX:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Box>(geom));
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Mesh>(geom), cow);
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Octree>(geom), cow);
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using BULLET yet", static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name,
                                               const int& type_id,
                                               const CollisionShapesConst& shapes,
                                               const VectorIsometry3d& shape_poses)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
{
  assert(!shapes.empty());
  assert(!shape_poses.empty());
  assert(!name.empty());
  assert(shapes.size() == shape_poses.size());

  m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
  m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;

  if (shapes.size() == 1 && m_shape_poses[0].matrix().isIdentity())
  {
    btCollisionShape* shape = createShapePrimitive(m_shapes[0], this);
    shape->setMargin(BULLET_MARGIN);
    manage(shape);
    setCollisionShape(shape);
  }
  else
  {
    btCompoundShape* compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
    manage(compound);
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative
    setCollisionShape(compound);

    for (std::size_t j = 0; j < m_shapes.size(); ++j)
    {
      btCollisionShape* subshape = createShapePrimitive(m_shapes[j], this);
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

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name,
                                               const int& type_id,
                                               const CollisionShapesConst& shapes,
                                               const VectorIsometry3d& shape_poses,
                                               const std::vector<std::shared_ptr<void>>& data)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_data(data)
{
}
}
}
