/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_convex_sdf_algorithm.h>
#include <tesseract/collision/implicit_sdf_collision_solver.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/capsule.h>
#include <tesseract/geometry/impl/cone.h>
#include <tesseract/geometry/impl/cylinder.h>
#include <tesseract/geometry/impl/sphere.h>

#include <limits>

namespace tesseract::collision::bullet_internal
{
namespace
{
ImplicitSDFShape makeBulletConvexShape(const btConvexShape& shape, const btTransform& world_transform)
{
  const Eigen::Isometry3d pose = convertBtToEigen(world_transform);
  switch (shape.getShapeType())
  {
    case BOX_SHAPE_PROXYTYPE:
    {
      const btVector3 half_extents = static_cast<const btBoxShape&>(shape).getHalfExtentsWithoutMargin();
      const tesseract::geometry::Box box(2.0 * static_cast<double>(half_extents.x()),
                                         2.0 * static_cast<double>(half_extents.y()),
                                         2.0 * static_cast<double>(half_extents.z()));
      return makeImplicitSDFShape(box, pose);
    }
    case SPHERE_SHAPE_PROXYTYPE:
    {
      const tesseract::geometry::Sphere sphere(
          static_cast<double>(static_cast<const btSphereShape&>(shape).getRadius()));
      return makeImplicitSDFShape(sphere, pose);
    }
    case CAPSULE_SHAPE_PROXYTYPE:
    {
      const auto& capsule_shape = static_cast<const btCapsuleShape&>(shape);
      const tesseract::geometry::Capsule capsule(static_cast<double>(capsule_shape.getRadius()),
                                                 2.0 * static_cast<double>(capsule_shape.getHalfHeight()));
      return makeImplicitSDFShape(capsule, pose);
    }
    case CYLINDER_SHAPE_PROXYTYPE:
    {
      const auto& cylinder_shape = static_cast<const btCylinderShape&>(shape);
      const tesseract::geometry::Cylinder cylinder(
          static_cast<double>(cylinder_shape.getRadius()),
          2.0 * static_cast<double>(cylinder_shape.getHalfExtentsWithoutMargin()[cylinder_shape.getUpAxis()]));
      return makeImplicitSDFShape(cylinder, pose);
    }
    case CONE_SHAPE_PROXYTYPE:
    {
      const auto& cone_shape = static_cast<const btConeShape&>(shape);
      const tesseract::geometry::Cone cone(static_cast<double>(cone_shape.getRadius()),
                                           static_cast<double>(cone_shape.getHeight()));
      return makeImplicitSDFShape(cone, pose);
    }
    default:
      return {};
  }
}

ImplicitSDFShape makeBulletSDFShape(btSdfCollisionShape& shape, const btTransform& world_transform)
{
  const btTransform inverse_transform = world_transform.inverse();
  ImplicitSDFShape result;
  result.distance = [&shape, inverse_transform](const Eigen::Vector3d& point) {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!shape.queryPoint(inverse_transform * convertEigenToBt(point), distance, normal))
      return std::numeric_limits<double>::infinity();
    return static_cast<double>(distance);
  };
  result.gradient = [&shape, inverse_transform, world_transform](const Eigen::Vector3d& point) -> Eigen::Vector3d {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!shape.queryPoint(inverse_transform * convertEigenToBt(point), distance, normal))
      return Eigen::Vector3d::Zero();
    normal = world_transform.getBasis() * normal;
    return convertBtToEigen(normal);
  };
  btVector3 aabb_min;
  btVector3 aabb_max;
  shape.getAabb(world_transform, aabb_min, aabb_max);
  result.aabb = Eigen::AlignedBox3d(convertBtToEigen(aabb_min), convertBtToEigen(aabb_max));
  return result;
}
}  // namespace

TesseractConvexSdfAlgorithm::TesseractConvexSdfAlgorithm(btPersistentManifold* mf,
                                                         const btCollisionAlgorithmConstructionInfo& ci,
                                                         const btCollisionObjectWrapper* /*body0Wrap*/,
                                                         const btCollisionObjectWrapper* /*body1Wrap*/,
                                                         bool isSwapped)
  : btActivatingCollisionAlgorithm(ci), m_isSwapped(isSwapped), m_manifoldPtr(mf)
{
}

TesseractConvexSdfAlgorithm::~TesseractConvexSdfAlgorithm()
{
  if (m_ownManifold && m_manifoldPtr != nullptr)
    m_dispatcher->releaseManifold(m_manifoldPtr);
}

void TesseractConvexSdfAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap,
                                                   const btDispatcherInfo& /*dispatchInfo*/,
                                                   btManifoldResult* resultOut)
{
  const btCollisionObjectWrapper* convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
  const btCollisionObjectWrapper* sdfBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;

  // queryPoint is a non-const member on btSdfCollisionShape even though it does not mutate the
  // field, hence the const_cast (stock Bullet C-casts away constness in the same spot).
  // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
  auto* sdf_shape =
      const_cast<btSdfCollisionShape*>(static_cast<const btSdfCollisionShape*>(sdfBodyWrap->getCollisionShape()));
  // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
  const auto* convex = static_cast<const btConvexShape*>(convexBodyWrap->getCollisionShape());

  const ImplicitSDFShape implicit_convex = makeBulletConvexShape(*convex, convexBodyWrap->getWorldTransform());
  if (implicit_convex.isValid())
  {
    const ImplicitSDFShape implicit_sdf = makeBulletSDFShape(*sdf_shape, sdfBodyWrap->getWorldTransform());
    ImplicitSDFCollisionConfig config;
    config.contact_margin = static_cast<double>(resultOut->m_closestPointDistanceThreshold);
    const std::vector<ImplicitSDFContact> contacts = collideImplicitSDF(implicit_convex, implicit_sdf, config);

    if (m_manifoldPtr == nullptr)
    {
      m_manifoldPtr =
          m_dispatcher->getNewManifold(convexBodyWrap->getCollisionObject(), sdfBodyWrap->getCollisionObject());
      m_ownManifold = true;
    }
    resultOut->setPersistentManifold(m_manifoldPtr);

    for (const auto& contact : contacts)
    {
      // Bullet expects the normal on body B (the SDF) directed toward body A, whereas the shared
      // solver returns the normal from shape 0 (the convex) toward shape 1 (the SDF).
      resultOut->addContactPoint(convertEigenToBt(Eigen::Vector3d(-contact.normal)),
                                 convertEigenToBt(contact.nearest_points[1]),
                                 static_cast<btScalar>(contact.distance));
    }
    resultOut->refreshContactPoints();
    return;
  }

  // Preserve stock Bullet's vertex-query behavior for arbitrary polyhedra that do not yet have a
  // backend-neutral signed-distance adapter.
  btAlignedObjectArray<btVector3> query_vertices;
  if (convex->isPolyhedral())
  {
    const auto* poly = static_cast<const btPolyhedralConvexShape*>(convex);
    for (int v = 0; v < poly->getNumVertices(); v++)
    {
      btVector3 vtx;
      poly->getVertex(v, vtx);
      query_vertices.push_back(vtx);
    }
  }

  if (query_vertices.size() == 0)
  {
    if (!m_warned_unsupported)
    {
      CONSOLE_BRIDGE_logWarn("Convex shape type '%s' cannot be collision checked against a signed distance field; "
                             "only boxes, spheres, capsules, cylinders, cones and polyhedral shapes are supported. "
                             "No contacts will be reported for this pair.",
                             convex->getName());
      m_warned_unsupported = true;
    }
    return;
  }

  if (m_manifoldPtr == nullptr)
  {
    m_manifoldPtr =
        m_dispatcher->getNewManifold(convexBodyWrap->getCollisionObject(), sdfBodyWrap->getCollisionObject());
    m_ownManifold = true;
  }
  resultOut->setPersistentManifold(m_manifoldPtr);

  const btScalar max_dist = resultOut->m_closestPointDistanceThreshold + SIMD_EPSILON;

  for (int v = 0; v < query_vertices.size(); v++)
  {
    const btVector3& vtx = query_vertices[v];
    btVector3 vtx_world_space = convexBodyWrap->getWorldTransform() * vtx;
    btVector3 vtx_in_sdf = sdfBodyWrap->getWorldTransform().invXform(vtx_world_space);

    btVector3 normal_local;
    btScalar dist{ 0 };
    if (sdf_shape->queryPoint(vtx_in_sdf, dist, normal_local))
    {
      if (dist <= max_dist)
      {
        normal_local.safeNormalize();
        btVector3 normal = sdfBodyWrap->getWorldTransform().getBasis() * normal_local;

        resultOut->addContactPoint(normal, vtx_world_space - normal * dist, dist);
      }
    }
  }
  resultOut->refreshContactPoints();
}

btScalar TesseractConvexSdfAlgorithm::calculateTimeOfImpact(btCollisionObject* /*body0*/,
                                                            btCollisionObject* /*body1*/,
                                                            const btDispatcherInfo& /*dispatchInfo*/,
                                                            btManifoldResult* /*resultOut*/)
{
  // Continuous collision checking against SDF geometry is not supported; the cast managers reject
  // SDF shapes at addCollisionObject time so this is never reached in practice.
  return btScalar(1.);
}

}  // namespace tesseract::collision::bullet_internal
