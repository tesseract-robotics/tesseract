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

namespace tesseract_collision::tesseract_collision_bullet
{
btVector3 convertEigenToBt(const Eigen::Vector3d& v)
{
  return btVector3{ static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2]) };
}

Eigen::Vector3d convertBtToEigen(const btVector3& v)
{
  return Eigen::Vector3d{ static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z()) };
}

btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
{
  return btQuaternion{ static_cast<btScalar>(q.x()),
                       static_cast<btScalar>(q.y()),
                       static_cast<btScalar>(q.z()),
                       static_cast<btScalar>(q.w()) };
}

btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
{
  return btMatrix3x3{ static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)), static_cast<btScalar>(r(0, 2)),
                      static_cast<btScalar>(r(1, 0)), static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
                      static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)), static_cast<btScalar>(r(2, 2)) };
}

Eigen::Matrix3d convertBtToEigen(const btMatrix3x3& r)
{
  Eigen::Matrix3d m;
  m << static_cast<double>(r[0][0]), static_cast<double>(r[0][1]), static_cast<double>(r[0][2]),
      static_cast<double>(r[1][0]), static_cast<double>(r[1][1]), static_cast<double>(r[1][2]),
      static_cast<double>(r[2][0]), static_cast<double>(r[2][1]), static_cast<double>(r[2][2]);
  return m;
}

btTransform convertEigenToBt(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix3d& rot = t.matrix().block<3, 3>(0, 0);
  const Eigen::Vector3d& tran = t.translation();

  return btTransform{ convertEigenToBt(rot), convertEigenToBt(tran) };
}

Eigen::Isometry3d convertBtToEigen(const btTransform& t)
{
  Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
  i.linear() = convertBtToEigen(t.getBasis());
  i.translation() = convertBtToEigen(t.getOrigin());

  return i;
}

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
      assert(triangles[4L * i] == 3);
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

  std::vector<std::shared_ptr<btCollisionShape>> managed_shapes;
  managed_shapes.resize(octree.getTreeDepth() + 1);
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

          std::shared_ptr<btCollisionShape> childshape = managed_shapes.at(it.getDepth());
          if (childshape == nullptr)
          {
            childshape = std::make_shared<btBoxShape>(btVector3(l, l, l));
            childshape->setUserIndex(shape_index);
            childshape->setMargin(BULLET_MARGIN);
            managed_shapes.at(it.getDepth()) = childshape;
          }

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }

      cow->manageReserve(managed_shapes.size());
      for (const auto& managed_shape : managed_shapes)
      {
        if (managed_shape != nullptr)
          cow->manage(managed_shape);
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

          std::shared_ptr<btCollisionShape> childshape = managed_shapes.at(it.getDepth());
          if (childshape == nullptr)
          {
            childshape = std::make_shared<btSphereShape>(static_cast<btScalar>((size / 2)));
            childshape->setUserIndex(shape_index);
            // Sphere is a special case where you do not modify the margin which is internally set to the radius
            managed_shapes.at(it.getDepth()) = childshape;
          }

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }

      cow->manageReserve(managed_shapes.size());
      for (const auto& managed_shape : managed_shapes)
      {
        if (managed_shape != nullptr)
          cow->manage(managed_shape);
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

          std::shared_ptr<btCollisionShape> childshape = managed_shapes.at(it.getDepth());
          if (childshape == nullptr)
          {
            childshape =
                std::make_shared<btSphereShape>(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
            childshape->setUserIndex(shape_index);
            // Sphere is a special case where you do not modify the margin which is internally set to the radius
            managed_shapes.at(it.getDepth()) = childshape;
          }

          subshape->addChildShape(geomTrans, childshape.get());
        }
      }

      cow->manageReserve(managed_shapes.size());
      for (const auto& managed_shape : managed_shapes)
      {
        if (managed_shape != nullptr)
          cow->manage(managed_shape);
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

void updateCollisionObjectFilters(const std::vector<std::string>& active, const COW::Ptr& cow)
{
  cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;

  if (!isLinkActive(active, cow->getName()))
  {
    cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
  }

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
  {
    cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
  }
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

const std::string& CollisionObjectWrapper::getName() const { return m_name; }

const int& CollisionObjectWrapper::getTypeID() const { return m_type_id; }

bool CollisionObjectWrapper::sameObject(const CollisionObjectWrapper& other) const
{
  return m_name == other.m_name && m_type_id == other.m_type_id && m_shapes.size() == other.m_shapes.size() &&
         m_shape_poses.size() == other.m_shape_poses.size() &&
         std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
         std::equal(m_shape_poses.begin(),
                    m_shape_poses.end(),
                    other.m_shape_poses.begin(),
                    [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
}

const CollisionShapesConst& CollisionObjectWrapper::getCollisionGeometries() const { return m_shapes; }

const tesseract_common::VectorIsometry3d& CollisionObjectWrapper::getCollisionGeometriesTransforms() const
{
  return m_shape_poses;
}

void CollisionObjectWrapper::getAABB(btVector3& aabb_min, btVector3& aabb_max) const
{
  getCollisionShape()->getAabb(getWorldTransform(), aabb_min, aabb_max);
  const btScalar& d = getContactProcessingThreshold();
  btVector3 contactThreshold(d, d, d);
  aabb_min -= contactThreshold;
  aabb_max += contactThreshold;
}

std::shared_ptr<CollisionObjectWrapper> CollisionObjectWrapper::clone()
{
  auto clone_cow = std::make_shared<CollisionObjectWrapper>();
  clone_cow->m_name = m_name;
  clone_cow->m_type_id = m_type_id;
  clone_cow->m_shapes = m_shapes;
  clone_cow->m_shape_poses = m_shape_poses;
  clone_cow->m_data = m_data;
  clone_cow->setCollisionShape(getCollisionShape());
  clone_cow->setWorldTransform(getWorldTransform());
  clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
  clone_cow->m_collisionFilterMask = m_collisionFilterMask;
  clone_cow->m_enabled = m_enabled;
  clone_cow->setBroadphaseHandle(nullptr);
  return clone_cow;
}

void CollisionObjectWrapper::manage(const std::shared_ptr<btCollisionShape>& t) { m_data.push_back(t); }

void CollisionObjectWrapper::manageReserve(std::size_t s) { m_data.reserve(s); }

CastHullShape::CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01)
{
  m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  setUserIndex(m_shape->getUserIndex());
}

void CastHullShape::updateCastTransform(const btTransform& t01) { m_t01 = t01; }
btVector3 CastHullShape::localGetSupportingVertex(const btVector3& vec) const
{
  btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
  btVector3 sv1 = m_t01 * m_shape->localGetSupportingVertex(vec * m_t01.getBasis());
  return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
}

/// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
void CastHullShape::getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const
{
  m_shape->getAabb(t_w0, aabbMin, aabbMax);
  btVector3 min1, max1;
  m_shape->getAabb(t_w0 * m_t01, min1, max1);
  aabbMin.setMin(min1);
  aabbMax.setMax(max1);
}

const char* CastHullShape::getName() const { return "CastHull"; }
btVector3 CastHullShape::localGetSupportingVertexWithoutMargin(const btVector3& v) const
{
  return localGetSupportingVertex(v);
}

// LCOV_EXCL_START
// notice that the vectors should be unit length
void CastHullShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* /*vectors*/,
                                                                      btVector3* /*supportVerticesOut*/,
                                                                      int /*numVectors*/) const
{
  throw std::runtime_error("If you are seeing this error message then something in Bullet must have changed. Attach "
                           "a debugger and inspect the call stack to find the function in Bullet calling this "
                           "function, then review commit history to determine what change.");
}

void CastHullShape::getAabbSlow(const btTransform& /*t*/, btVector3& /*aabbMin*/, btVector3& /*aabbMax*/) const
{
  throw std::runtime_error("If you are seeing this error message then something in Bullet must have changed. Attach "
                           "a debugger and inspect the call stack to find the function in Bullet calling this "
                           "function, then review commit history to determine what change.");
}

void CastHullShape::setLocalScaling(const btVector3& /*scaling*/)
{
  throw std::runtime_error("If you are seeing this error message then something in Bullet must have changed. Attach "
                           "a debugger and inspect the call stack to find the function in Bullet calling this "
                           "function, then review commit history to determine what change.");
}

const btVector3& CastHullShape::getLocalScaling() const
{
  static btVector3 out(1, 1, 1);
  return out;
}

void CastHullShape::setMargin(btScalar /*margin*/) {}

btScalar CastHullShape::getMargin() const { return 0; }

int CastHullShape::getNumPreferredPenetrationDirections() const { return 0; }

void CastHullShape::getPreferredPenetrationDirection(int /*index*/, btVector3& /*penetrationVector*/) const
{
  throw std::runtime_error("If you are seeing this error message then something in Bullet must have changed. Attach "
                           "a debugger and inspect the call stack to find the function in Bullet calling this "
                           "function, then review commit history to determine what change.");
}

void CastHullShape::calculateLocalInertia(btScalar, btVector3&) const
{
  throw std::runtime_error("If you are seeing this error message then something in Bullet must have changed. Attach "
                           "a debugger and inspect the call stack to find the function in Bullet calling this "
                           "function, then review commit history to determine what change.");
}

void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, btScalar& outsupport, btVector3& outpt)
{
  btVector3 ptSum(0, 0, 0);
  btScalar ptCount = 0;
  btScalar maxSupport = -1000;

  const auto* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape != nullptr)
  {
    int nPts = pshape->getNumVertices();

    for (int i = 0; i < nPts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      btScalar sup = pt.dot(localNormal);
      if (sup > maxSupport + BULLET_EPSILON)
      {
        ptCount = 1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - BULLET_EPSILON)
      {
      }
      else
      {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else
  {
    // The margins are set to zero for most shapes, but for a sphere the margin is used so must use
    // localGetSupportingVertex instead of localGetSupportingVertexWithoutMargin.
    outpt = shape->localGetSupportingVertex(localNormal);  // NOLINT
    outsupport = localNormal.dot(outpt);
  }
}

btTransform getLinkTransformFromCOW(const btCollisionObjectWrapper* cow)
{
  if (cow->m_parent != nullptr)
  {
    if (cow->m_parent->m_parent != nullptr)
    {
      assert(cow->m_parent->m_parent->m_parent == nullptr);
      return cow->m_parent->m_parent->getWorldTransform();
    }

    return cow->m_parent->getWorldTransform();
  }

  return cow->getWorldTransform();
}

bool needsCollisionCheck(const COW& cow1, const COW& cow2, const IsContactAllowedFn& acm, bool verbose)
{
  return cow1.m_enabled && cow2.m_enabled && (cow2.m_collisionFilterGroup & cow1.m_collisionFilterMask) &&  // NOLINT
         (cow1.m_collisionFilterGroup & cow2.m_collisionFilterMask) &&                                      // NOLINT
         !isContactAllowed(cow1.getName(), cow2.getName(), acm, verbose);
}

btScalar addDiscreteSingleResult(btManifoldPoint& cp,
                                 const btCollisionObjectWrapper* colObj0Wrap,
                                 const btCollisionObjectWrapper* colObj1Wrap,
                                 ContactTestData& collisions)
{
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);
  const auto* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());  // NOLINT
  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());  // NOLINT

  ObjectPairKey pc = getObjectPairKey(cd0->getName(), cd1->getName());

  const auto& it = collisions.res->find(pc);
  bool found = (it != collisions.res->end());

  //    size_t l = 0;
  //    if (found)
  //    {
  //      l = it->second.size();
  //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
  //          return 0;

  //    }

  btTransform tf0 = getLinkTransformFromCOW(colObj0Wrap);
  btTransform tf1 = getLinkTransformFromCOW(colObj1Wrap);
  btTransform tf0_inv = tf0.inverse();
  btTransform tf1_inv = tf1.inverse();

  ContactResult contact;
  contact.link_names[0] = cd0->getName();
  contact.link_names[1] = cd1->getName();
  contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
  contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
  contact.subshape_id[0] = colObj0Wrap->m_index;
  contact.subshape_id[1] = colObj1Wrap->m_index;
  contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
  contact.nearest_points_local[0] = convertBtToEigen(tf0_inv * cp.m_positionWorldOnA);
  contact.nearest_points_local[1] = convertBtToEigen(tf1_inv * cp.m_positionWorldOnB);
  contact.transform[0] = convertBtToEigen(tf0);
  contact.transform[1] = convertBtToEigen(tf1);
  contact.type_id[0] = cd0->getTypeID();
  contact.type_id[1] = cd1->getTypeID();
  contact.distance = static_cast<double>(cp.m_distance1);
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

  if (processResult(collisions, contact, pc, found) == nullptr)
    return 0;

  return 1;
}

void calculateContinuousData(ContactResult* col,
                             const btCollisionObjectWrapper* cow,
                             const btVector3& pt_world,
                             const btVector3& normal_world,
                             const btTransform& link_tf_inv,
                             size_t link_index)
{
  assert(dynamic_cast<const CastHullShape*>(cow->getCollisionShape()) != nullptr);
  const auto* shape = static_cast<const CastHullShape*>(cow->getCollisionShape());
  assert(shape != nullptr);

  // Get the start and final location of the shape
  btTransform shape_tfWorld0 = cow->getWorldTransform();
  btTransform shape_tfWorld1 = cow->getWorldTransform() * shape->m_t01;

  // Given the shapes final location calculate the links transform at the final location
  // NOLINTNEXTLINE
  Eigen::Isometry3d s = col->transform[link_index].inverse() * convertBtToEigen(shape_tfWorld0);
  col->cc_transform[link_index] = convertBtToEigen(shape_tfWorld1) * s.inverse();

  // Get the normal in the local shapes coordinate system at start and final location
  btVector3 shape_normalLocal0 = normal_world * shape_tfWorld0.getBasis();
  btVector3 shape_normalLocal1 = normal_world * shape_tfWorld1.getBasis();

  // Calculate the contact point at the start location using the casted normal vector in thapes local coordinate system
  btVector3 shape_ptLocal0;
  btScalar shape_localsup0{ std::numeric_limits<btScalar>::max() };
  GetAverageSupport(shape->m_shape, shape_normalLocal0, shape_localsup0, shape_ptLocal0);
  btVector3 shape_ptWorld0 = shape_tfWorld0 * shape_ptLocal0;

  // Calculate the contact point at the final location using the casted normal vector in thapes local coordinate system
  btVector3 shape_ptLocal1;
  btScalar shape_localsup1{ std::numeric_limits<btScalar>::max() };
  GetAverageSupport(shape->m_shape, shape_normalLocal1, shape_localsup1, shape_ptLocal1);
  btVector3 shape_ptWorld1 = shape_tfWorld1 * shape_ptLocal1;

  btScalar shape_sup0 = normal_world.dot(shape_ptWorld0);
  btScalar shape_sup1 = normal_world.dot(shape_ptWorld1);

  // TODO: this section is potentially problematic. think hard about the math
  if (shape_sup0 - shape_sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    // LCOV_EXCL_START
    col->cc_time[link_index] = 0;
    col->cc_type[link_index] = ContinuousCollisionType::CCType_Time0;
    // LCOV_EXCL_STOP
  }
  else if (shape_sup1 - shape_sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    // LCOV_EXCL_START
    col->cc_time[link_index] = 1;
    col->cc_type[link_index] = ContinuousCollisionType::CCType_Time1;
    // LCOV_EXCL_STOP
  }
  else
  {
    // Given the contact point at the start and final location along with the casted contact point
    // the time between 0 and 1 can be calculated along the path between the start and final location contact occurs.
    btScalar l0c = (pt_world - shape_ptWorld0).length();
    btScalar l1c = (pt_world - shape_ptWorld1).length();

    col->nearest_points_local[link_index] =
        convertBtToEigen(link_tf_inv * (shape_tfWorld0 * ((shape_ptLocal0 + shape_ptLocal1) / 2.0)));
    col->cc_type[link_index] = ContinuousCollisionType::CCType_Between;

    if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
    {
      col->cc_time[link_index] = .5;  // LCOV_EXCL_LINE
    }
    else
    {
      col->cc_time[link_index] = static_cast<double>(l0c / (l0c + l1c));
    }
  }
}

btScalar addCastSingleResult(btManifoldPoint& cp,
                             const btCollisionObjectWrapper* colObj0Wrap,
                             int /*index0*/,
                             const btCollisionObjectWrapper* colObj1Wrap,
                             int /*index1*/,
                             ContactTestData& collisions)
{
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);
  const auto* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());  // NOLINT
  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());  // NOLINT

  const std::pair<std::string, std::string>& pc = cd0->getName() < cd1->getName() ?
                                                      std::make_pair(cd0->getName(), cd1->getName()) :
                                                      std::make_pair(cd1->getName(), cd0->getName());

  auto it = collisions.res->find(pc);
  bool found = it != collisions.res->end();

  //    size_t l = 0;
  //    if (found)
  //    {
  //      l = it->second.size();
  //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
  //          return 0;
  //    }

  btTransform tf0 = getLinkTransformFromCOW(colObj0Wrap);
  btTransform tf1 = getLinkTransformFromCOW(colObj1Wrap);
  btTransform tf0_inv = tf0.inverse();
  btTransform tf1_inv = tf1.inverse();

  ContactResult contact;
  contact.link_names[0] = cd0->getName();
  contact.link_names[1] = cd1->getName();
  contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
  contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
  contact.subshape_id[0] = colObj0Wrap->m_index;
  contact.subshape_id[1] = colObj1Wrap->m_index;
  contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
  contact.nearest_points_local[0] = convertBtToEigen(tf0_inv * cp.m_positionWorldOnA);
  contact.nearest_points_local[1] = convertBtToEigen(tf1_inv * cp.m_positionWorldOnB);
  contact.transform[0] = convertBtToEigen(tf0);
  contact.transform[1] = convertBtToEigen(tf1);
  contact.type_id[0] = cd0->getTypeID();
  contact.type_id[1] = cd1->getTypeID();
  contact.distance = static_cast<double>(cp.m_distance1);
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

  ContactResult* col = processResult(collisions, contact, pc, found);
  if (col == nullptr)
    return 0;

  if (cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter &&
      cd1->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
  {
    calculateContinuousData(col, colObj0Wrap, cp.m_positionWorldOnA, -1 * cp.m_normalWorldOnB, tf0_inv, 0);
    calculateContinuousData(col, colObj1Wrap, cp.m_positionWorldOnB, cp.m_normalWorldOnB, tf1_inv, 1);
  }
  else
  {
    bool castShapeIsFirst = (cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);
    btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
    const btCollisionObjectWrapper* firstColObjWrap = (castShapeIsFirst ? colObj0Wrap : colObj1Wrap);
    const btTransform& first_tf_inv = (castShapeIsFirst ? tf0_inv : tf1_inv);
    const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;

    if (castShapeIsFirst)
    {
      std::swap(col->nearest_points[0], col->nearest_points[1]);
      std::swap(col->nearest_points_local[0], col->nearest_points_local[1]);
      std::swap(col->transform[0], col->transform[1]);
      std::swap(col->link_names[0], col->link_names[1]);
      std::swap(col->type_id[0], col->type_id[1]);
      std::swap(col->shape_id[0], col->shape_id[1]);
      std::swap(col->subshape_id[0], col->subshape_id[1]);
      col->normal *= -1;
    }

    calculateContinuousData(col, firstColObjWrap, ptOnCast, normalWorldFromCast, first_tf_inv, 1);
  }

  return 1;
}

TesseractBridgedManifoldResult::TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                                               const btCollisionObjectWrapper* obj1Wrap,
                                                               btCollisionWorld::ContactResultCallback& resultCallback)
  : btManifoldResult(obj0Wrap, obj1Wrap), m_resultCallback(resultCallback)
{
}

void TesseractBridgedManifoldResult::addContactPoint(const btVector3& normalOnBInWorld,
                                                     const btVector3& pointInWorld,
                                                     btScalar depth)
{
  bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
  btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
  btVector3 localA;
  btVector3 localB;
  if (isSwapped)
  {
    localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
    localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
  }
  else
  {
    localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
    localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
  }

  btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
  newPt.m_positionWorldOnA = pointA;
  newPt.m_positionWorldOnB = pointInWorld;

  // BP mod, store contact triangles.
  if (isSwapped)
  {
    newPt.m_partId0 = m_partId1;
    newPt.m_partId1 = m_partId0;
    newPt.m_index0 = m_index1;
    newPt.m_index1 = m_index0;
  }
  else
  {
    newPt.m_partId0 = m_partId0;
    newPt.m_partId1 = m_partId1;
    newPt.m_index0 = m_index0;
    newPt.m_index1 = m_index1;
  }

  // experimental feature info, for per-triangle material etc.
  const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
  const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
  m_resultCallback.addSingleResult(
      newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
}

BroadphaseContactResultCallback::BroadphaseContactResultCallback(ContactTestData& collisions,
                                                                 double contact_distance,
                                                                 bool verbose)
  : collisions_(collisions), contact_distance_(contact_distance), verbose_(verbose)
{
}

bool BroadphaseContactResultCallback::needsCollision(const CollisionObjectWrapper* cow0,
                                                     const CollisionObjectWrapper* cow1) const
{
  return !collisions_.done && needsCollisionCheck(*cow0, *cow1, collisions_.fn, verbose_);
}

DiscreteBroadphaseContactResultCallback::DiscreteBroadphaseContactResultCallback(ContactTestData& collisions,
                                                                                 double contact_distance,
                                                                                 bool verbose)
  : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
{
}

btScalar DiscreteBroadphaseContactResultCallback::addSingleResult(btManifoldPoint& cp,
                                                                  const btCollisionObjectWrapper* colObj0Wrap,
                                                                  int /*partId0*/,
                                                                  int /*index0*/,
                                                                  const btCollisionObjectWrapper* colObj1Wrap,
                                                                  int /*partId1*/,
                                                                  int /*index1*/)
{
  if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    return 0;

  return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
}

CastBroadphaseContactResultCallback::CastBroadphaseContactResultCallback(ContactTestData& collisions,
                                                                         double contact_distance,
                                                                         bool verbose)
  : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
{
}

btScalar CastBroadphaseContactResultCallback::addSingleResult(btManifoldPoint& cp,
                                                              const btCollisionObjectWrapper* colObj0Wrap,
                                                              int /*partId0*/,
                                                              int index0,
                                                              const btCollisionObjectWrapper* colObj1Wrap,
                                                              int /*partId1*/,
                                                              int index1)
{
  if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    return 0;

  return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
}

TesseractBroadphaseBridgedManifoldResult::TesseractBroadphaseBridgedManifoldResult(
    const btCollisionObjectWrapper* obj0Wrap,
    const btCollisionObjectWrapper* obj1Wrap,
    BroadphaseContactResultCallback& result_callback)
  : btManifoldResult(obj0Wrap, obj1Wrap), result_callback_(result_callback)
{
}

void TesseractBroadphaseBridgedManifoldResult::addContactPoint(const btVector3& normalOnBInWorld,
                                                               const btVector3& pointInWorld,
                                                               btScalar depth)
{
  if (result_callback_.collisions_.done || depth > static_cast<btScalar>(result_callback_.contact_distance_))
    return;

  bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
  btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
  btVector3 localA;
  btVector3 localB;
  if (isSwapped)
  {
    localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
    localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
  }
  else
  {
    localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
    localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
  }

  btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
  newPt.m_positionWorldOnA = pointA;
  newPt.m_positionWorldOnB = pointInWorld;

  // BP mod, store contact triangles.
  if (isSwapped)
  {
    newPt.m_partId0 = m_partId1;
    newPt.m_partId1 = m_partId0;
    newPt.m_index0 = m_index1;
    newPt.m_index1 = m_index0;
  }
  else
  {
    newPt.m_partId0 = m_partId0;
    newPt.m_partId1 = m_partId1;
    newPt.m_index0 = m_index0;
    newPt.m_index1 = m_index1;
  }

  // experimental feature info, for per-triangle material etc.
  const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
  const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
  result_callback_.addSingleResult(
      newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
}

TesseractCollisionPairCallback::TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo,
                                                               btCollisionDispatcher* dispatcher,
                                                               BroadphaseContactResultCallback& results_callback)
  : dispatch_info_(dispatchInfo), dispatcher_(dispatcher), results_callback_(results_callback)
{
}

bool TesseractCollisionPairCallback::processOverlap(btBroadphasePair& pair)
{
  if (results_callback_.collisions_.done)
    return false;

  const auto* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
  const auto* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

  if (results_callback_.needsCollision(cow0, cow1))
  {
    btCollisionObjectWrapper obj0Wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
    btCollisionObjectWrapper obj1Wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

    // dispatcher will keep algorithms persistent in the collision pair
    if (pair.m_algorithm == nullptr)
    {
      pair.m_algorithm = dispatcher_->findAlgorithm(&obj0Wrap, &obj1Wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
    }

    if (pair.m_algorithm != nullptr)
    {
      TesseractBroadphaseBridgedManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap, results_callback_);
      contactPointResult.m_closestPointDistanceThreshold = static_cast<btScalar>(results_callback_.contact_distance_);

      // discrete collision detection query
      pair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatch_info_, &contactPointResult);
    }
  }
  return false;
}

TesseractOverlapFilterCallback::TesseractOverlapFilterCallback(bool verbose) : verbose_(verbose) {}

bool TesseractOverlapFilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
{
  // Note: We do not pass the allowed collision matrix because if it changes we do not know and this function only
  // gets called under certain cases and it could cause overlapping pairs to not be processed.
  return needsCollisionCheck(*(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)),
                             *(static_cast<CollisionObjectWrapper*>(proxy1->m_clientObject)),
                             nullptr,
                             verbose_);
}

COW::Ptr createCollisionObject(const std::string& name,
                               const int& type_id,
                               const CollisionShapesConst& shapes,
                               const tesseract_common::VectorIsometry3d& shape_poses,
                               bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  auto new_cow = std::make_shared<COW>(name, type_id, shapes, shape_poses);

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

DiscreteCollisionCollector::DiscreteCollisionCollector(ContactTestData& collisions,
                                                       COW::Ptr cow,
                                                       btScalar contact_distance,
                                                       bool verbose)
  : collisions_(collisions), cow_(std::move(cow)), contact_distance_(contact_distance), verbose_(verbose)
{
  m_closestDistanceThreshold = contact_distance;
  m_collisionFilterGroup = cow_->m_collisionFilterGroup;
  m_collisionFilterMask = cow_->m_collisionFilterMask;
}

btScalar DiscreteCollisionCollector::addSingleResult(btManifoldPoint& cp,
                                                     const btCollisionObjectWrapper* colObj0Wrap,
                                                     int /*partId0*/,
                                                     int /*index0*/,
                                                     const btCollisionObjectWrapper* colObj1Wrap,
                                                     int /*partId1*/,
                                                     int /*index1*/)
{
  if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    return 0;

  return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
}

bool DiscreteCollisionCollector::needsCollision(btBroadphaseProxy* proxy0) const
{
  return !collisions_.done &&
         needsCollisionCheck(
             *cow_, *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)), collisions_.fn, verbose_);
}

CastCollisionCollector::CastCollisionCollector(ContactTestData& collisions,
                                               COW::Ptr cow,
                                               double contact_distance,
                                               bool verbose)
  : collisions_(collisions), cow_(std::move(cow)), contact_distance_(contact_distance), verbose_(verbose)
{
  m_closestDistanceThreshold = static_cast<btScalar>(contact_distance);
  m_collisionFilterGroup = cow_->m_collisionFilterGroup;
  m_collisionFilterMask = cow_->m_collisionFilterMask;
}

btScalar CastCollisionCollector::addSingleResult(btManifoldPoint& cp,
                                                 const btCollisionObjectWrapper* colObj0Wrap,
                                                 int /*partId0*/,
                                                 int index0,
                                                 const btCollisionObjectWrapper* colObj1Wrap,
                                                 int /*partId1*/,
                                                 int index1)
{
  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    return 0;

  return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
}

bool CastCollisionCollector::needsCollision(btBroadphaseProxy* proxy0) const
{
  return !collisions_.done &&
         needsCollisionCheck(
             *cow_, *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)), collisions_.fn, verbose_);
}

COW::Ptr makeCastCollisionObject(const COW::Ptr& cow)
{
  COW::Ptr new_cow = cow->clone();

  btTransform tf;
  tf.setIdentity();

  if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
  {
    assert(dynamic_cast<btConvexShape*>(new_cow->getCollisionShape()) != nullptr);
    auto* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());  // NOLINT
    assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if the collision object is already a
                                                                 // cast collision object

    auto shape = std::make_shared<CastHullShape>(convex, tf);
    assert(shape != nullptr);

    new_cow->manage(shape);
    new_cow->setCollisionShape(shape.get());
  }
  else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
  {
    assert(dynamic_cast<btCompoundShape*>(new_cow->getCollisionShape()) != nullptr);
    auto* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());  // NOLINT
    auto new_compound =
        std::make_shared<btCompoundShape>(BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

    for (int i = 0; i < compound->getNumChildShapes(); ++i)
    {
      if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
      {
        auto* convex = static_cast<btConvexShape*>(compound->getChildShape(i));  // NOLINT
        assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

        btTransform geomTrans = compound->getChildTransform(i);

        auto subshape = std::make_shared<CastHullShape>(convex, tf);
        assert(subshape != nullptr);

        new_cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geomTrans, subshape.get());
      }
      else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
      {
        auto* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));  // NOLINT
        auto new_second_compound =
            std::make_shared<btCompoundShape>(BULLET_COMPOUND_USE_DYNAMIC_AABB, second_compound->getNumChildShapes());
        for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
        {
          assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));
          assert(dynamic_cast<btConvexShape*>(second_compound->getChildShape(j)) != nullptr);

          auto* convex = static_cast<btConvexShape*>(second_compound->getChildShape(j));  // NOLINT
          assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

          btTransform geomTrans = second_compound->getChildTransform(j);

          auto subshape = std::make_shared<CastHullShape>(convex, tf);
          assert(subshape != nullptr);

          new_cow->manage(subshape);
          subshape->setMargin(BULLET_MARGIN);
          new_second_compound->addChildShape(geomTrans, subshape.get());
        }

        btTransform geomTrans = compound->getChildTransform(i);

        new_cow->manage(new_second_compound);
        new_second_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                                        // have no effect when positive
                                                        // but has an effect when
                                                        // negative

        new_compound->addChildShape(geomTrans, new_second_compound.get());
      }
      else
      {
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }

    new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                             // have no effect when positive
                                             // but has an effect when
                                             // negative
    new_cow->manage(new_compound);
    new_cow->setCollisionShape(new_compound.get());
    new_cow->setWorldTransform(cow->getWorldTransform());
  }
  else
  {
    throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
  }

  return new_cow;
}

void updateBroadphaseAABB(const COW::Ptr& cow,
                          const std::unique_ptr<btBroadphaseInterface>& broadphase,
                          const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  // Calculate the aabb
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  // Update the broadphase aabb
  assert(cow->getBroadphaseHandle() != nullptr);
  broadphase->setAabb(cow->getBroadphaseHandle(), aabb_min, aabb_max, dispatcher.get());
}

void removeCollisionObjectFromBroadphase(const COW::Ptr& cow,
                                         const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                         const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btBroadphaseProxy* bp = cow->getBroadphaseHandle();
  if (bp != nullptr)
  {
    // only clear the cached algorithms
    broadphase->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher.get());
    broadphase->destroyProxy(bp, dispatcher.get());
    cow->setBroadphaseHandle(nullptr);
  }
}

void addCollisionObjectToBroadphase(const COW::Ptr& cow,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  // Add the active collision object to the broadphase
  int type{ cow->getCollisionShape()->getShapeType() };
  cow->setBroadphaseHandle(broadphase->createProxy(
      aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup, cow->m_collisionFilterMask, dispatcher.get()));
}

void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                  const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  updateCollisionObjectFilters(active, cow);

  // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
  // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
  // this must be called or contacts between shapes will be missed.
  broadphase->getOverlappingPairCache()->cleanProxyFromPairs(cow->getBroadphaseHandle(), dispatcher.get());
}

void refreshBroadphaseProxy(const COW::Ptr& cow,
                            const std::unique_ptr<btBroadphaseInterface>& broadphase,
                            const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  if (cow->getBroadphaseHandle() != nullptr)
  {
    broadphase->destroyProxy(cow->getBroadphaseHandle(), dispatcher.get());

    btVector3 aabb_min, aabb_max;
    cow->getAABB(aabb_min, aabb_max);

    // Add the active collision object to the broadphase
    int type{ cow->getCollisionShape()->getShapeType() };
    cow->setBroadphaseHandle(broadphase->createProxy(aabb_min,
                                                     aabb_max,
                                                     type,
                                                     cow.get(),
                                                     cow->m_collisionFilterGroup,
                                                     cow->m_collisionFilterMask,
                                                     dispatcher.get()));
  }
}
}  // namespace tesseract_collision::tesseract_collision_bullet
