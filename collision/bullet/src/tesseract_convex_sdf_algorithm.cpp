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
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <console_bridge/console.h>
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/tesseract_convex_sdf_algorithm.h>

namespace tesseract::collision::bullet_internal
{
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

  // This mirrors the SDF special case in btConvexConcaveCollisionAlgorithm::processCollision: the
  // field is sampled at the convex shape's vertices (or a sphere's center). The difference is that
  // the acceptance band below is widened by m_closestPointDistanceThreshold so separated contacts
  // within the pair's contact distance are reported.
  btAlignedObjectArray<btVector3> query_vertices;
  btScalar query_offset = 0;

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
  else if (convex->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
  {
    query_vertices.push_back(btVector3(0, 0, 0));
    const auto* sphere = static_cast<const btSphereShape*>(convex);
    query_offset = sphere->getRadius();
  }
  else if (convex->getShapeType() == CAPSULE_SHAPE_PROXYTYPE)
  {
    // A capsule is its axis segment swept by a sphere, so querying the field along the segment
    // with a radius offset is exact at each sample (same as the sphere case). The endpoints cover
    // the caps exactly; interior samples spaced at most one radius apart bound the error along the
    // barrel by the field's curvature between samples.
    const auto* capsule = static_cast<const btCapsuleShape*>(convex);
    const btScalar radius = capsule->getRadius();
    const btScalar half_height = capsule->getHalfHeight();
    query_offset = radius;

    auto num_intervals = static_cast<int>(std::ceil((btScalar(2.) * half_height) / radius));
    num_intervals = btMax(1, btMin(num_intervals, 64));
    for (int i = 0; i <= num_intervals; i++)
    {
      btVector3 vtx(0, 0, 0);
      vtx[capsule->getUpAxis()] = -half_height + (btScalar(2.) * half_height) * (btScalar(i) / btScalar(num_intervals));
      query_vertices.push_back(vtx);
    }
  }

  if (query_vertices.size() == 0)
  {
    // Same limitation as stock Bullet: only shapes that expose query points can be tested against
    // an SDF, so cylinders and cones silently miss every contact. Fail loudly instead, once per
    // pair (this method runs every contact test).
    if (!m_warned_unsupported)
    {
      CONSOLE_BRIDGE_logWarn("Convex shape type '%s' cannot be collision checked against a signed distance field; "
                             "only spheres, capsules and polyhedral shapes are supported. No contacts will be "
                             "reported for this pair. Consider approximating this shape with a capsule or a convex "
                             "hull instead.",
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

  const btScalar max_dist = query_offset + resultOut->m_closestPointDistanceThreshold + SIMD_EPSILON;

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

        if (query_offset > 0)
        {
          dist -= query_offset;
          vtx_world_space -= query_offset * normal;
        }
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
