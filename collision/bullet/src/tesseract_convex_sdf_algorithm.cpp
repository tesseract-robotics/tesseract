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
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <console_bridge/console.h>
#include <cmath>
#include <functional>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/tesseract_convex_sdf_algorithm.h>

namespace tesseract::collision::bullet_internal
{
namespace
{
constexpr int MIN_ANGULAR_INTERVALS = 8;
constexpr int MAX_ANGULAR_INTERVALS = 64;
constexpr int MAX_LINEAR_INTERVALS = 64;
constexpr btScalar SURFACE_INTERVALS_PER_MIN_DIMENSION = btScalar(8.);

/** @brief A rectangular parameterization of one exact primitive surface patch. */
struct ConvexSurfacePatch
{
  std::function<btVector3(btScalar, btScalar)> evaluate;
  btScalar u_length;
  btScalar v_length;
  bool u_is_periodic;
};

int getIntervalCount(btScalar length, btScalar target_spacing, int minimum, int maximum)
{
  if (length <= SIMD_EPSILON || target_spacing <= SIMD_EPSILON)
    return minimum;

  return btMax(minimum, btMin(static_cast<int>(std::ceil(length / target_spacing)), maximum));
}

void appendUniquePoint(btAlignedObjectArray<btVector3>& points, const btVector3& point)
{
  constexpr auto duplicate_tolerance_squared = btScalar(1e-12);
  for (int i = 0; i < points.size(); ++i)
  {
    if (points[i].distance2(point) <= duplicate_tolerance_squared)
      return;
  }
  points.push_back(point);
}

/**
 * @brief Sample a primitive surface patch with density selected from its physical dimensions.
 *
 * The parameterization keeps generated points on the exact primitive surface. Interval counts
 * adapt independently to each patch dimension and are bounded to keep narrowphase cost finite.
 */
void sampleSurfacePatch(const ConvexSurfacePatch& patch,
                        btScalar target_spacing,
                        btAlignedObjectArray<btVector3>& points)
{
  const int u_intervals = getIntervalCount(
      patch.u_length, target_spacing, patch.u_is_periodic ? MIN_ANGULAR_INTERVALS : 1, MAX_ANGULAR_INTERVALS);
  const int v_intervals = getIntervalCount(patch.v_length, target_spacing, 1, MAX_LINEAR_INTERVALS);
  const int u_samples = patch.u_is_periodic ? u_intervals : u_intervals + 1;

  for (int v = 0; v <= v_intervals; ++v)
  {
    const btScalar v_parameter = btScalar(v) / btScalar(v_intervals);
    for (int u = 0; u < u_samples; ++u)
    {
      const btScalar u_parameter = btScalar(u) / btScalar(u_intervals);
      appendUniquePoint(points, patch.evaluate(u_parameter, v_parameter));
    }
  }
}

std::pair<int, int> getRadialAxes(int up_axis)
{
  switch (up_axis)
  {
    case 0:
      return { 1, 2 };
    case 1:
      return { 0, 2 };
    default:
      return { 0, 1 };
  }
}

void appendCylinderSurfaceSamples(const btCylinderShape& cylinder, btAlignedObjectArray<btVector3>& points)
{
  const int up_axis = cylinder.getUpAxis();
  const auto radial_axes = getRadialAxes(up_axis);
  const int radial_axis_1 = radial_axes.first;
  const int radial_axis_2 = radial_axes.second;
  const btScalar radius = cylinder.getRadius();
  const btScalar half_height = cylinder.getHalfExtentsWithoutMargin()[up_axis];
  const btScalar target_spacing = btMin(radius, btScalar(2.) * half_height) / SURFACE_INTERVALS_PER_MIN_DIMENSION;

  const auto make_point = [=](btScalar radial_distance, btScalar angle, btScalar height) {
    btVector3 point(0, 0, 0);
    point[radial_axis_1] = radial_distance * btCos(angle);
    point[radial_axis_2] = radial_distance * btSin(angle);
    point[up_axis] = height;
    return point;
  };

  sampleSurfacePatch({ [=](btScalar u, btScalar v) {
                        return make_point(radius, SIMD_2_PI * u, -half_height + (btScalar(2.) * half_height * v));
                      },
                       SIMD_2_PI * radius,
                       btScalar(2.) * half_height,
                       true },
                     target_spacing,
                     points);

  for (const btScalar height : { -half_height, half_height })
  {
    sampleSurfacePatch({ [=](btScalar u, btScalar v) { return make_point(radius * v, SIMD_2_PI * u, height); },
                         SIMD_2_PI * radius,
                         radius,
                         true },
                       target_spacing,
                       points);
  }
}

void appendConeSurfaceSamples(const btConeShape& cone, btAlignedObjectArray<btVector3>& points)
{
  const int up_axis = cone.getConeUpIndex();
  const auto radial_axes = getRadialAxes(up_axis);
  const int radial_axis_1 = radial_axes.first;
  const int radial_axis_2 = radial_axes.second;
  const btScalar radius = cone.getRadius();
  const btScalar height = cone.getHeight();
  const btScalar half_height = height / btScalar(2.);
  const btScalar slant_height = btSqrt((radius * radius) + (height * height));
  const btScalar target_spacing = btMin(radius, height) / SURFACE_INTERVALS_PER_MIN_DIMENSION;

  const auto make_point = [=](btScalar radial_distance, btScalar angle, btScalar axial_position) {
    btVector3 point(0, 0, 0);
    point[radial_axis_1] = radial_distance * btCos(angle);
    point[radial_axis_2] = radial_distance * btSin(angle);
    point[up_axis] = axial_position;
    return point;
  };

  sampleSurfacePatch({ [=](btScalar u, btScalar v) {
                        return make_point(radius * (btScalar(1.) - v), SIMD_2_PI * u, -half_height + (height * v));
                      },
                       SIMD_2_PI * radius,
                       slant_height,
                       true },
                     target_spacing,
                     points);

  sampleSurfacePatch({ [=](btScalar u, btScalar v) { return make_point(radius * v, SIMD_2_PI * u, -half_height); },
                       SIMD_2_PI * radius,
                       radius,
                       true },
                     target_spacing,
                     points);
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
  else if (convex->getShapeType() == CYLINDER_SHAPE_PROXYTYPE)
  {
    appendCylinderSurfaceSamples(*static_cast<const btCylinderShape*>(convex), query_vertices);
  }
  else if (convex->getShapeType() == CONE_SHAPE_PROXYTYPE)
  {
    appendConeSurfaceSamples(*static_cast<const btConeShape*>(convex), query_vertices);
  }

  if (query_vertices.size() == 0)
  {
    // Same limitation as stock Bullet: only shapes that expose query points or have an explicit
    // surface sampler can be tested against an SDF. Fail loudly instead, once per pair (this
    // method runs every contact test).
    if (!m_warned_unsupported)
    {
      CONSOLE_BRIDGE_logWarn("Convex shape type '%s' cannot be collision checked against a signed distance field; "
                             "only spheres, capsules, cylinders, cones and polyhedral shapes are supported. No "
                             "contacts will be reported for this pair. Consider approximating this shape with a "
                             "capsule or a convex hull instead.",
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
