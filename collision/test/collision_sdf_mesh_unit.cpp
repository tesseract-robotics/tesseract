/**
 * @file collision_sdf_mesh_unit.cpp
 * @brief Collision tests for the two-in-one SDFMesh (triangle surface + optional discretized field).
 *
 * Verifies the backend routing added when SDFMesh gained a SignedDistanceField member:
 *   - Bullet discrete uses the attached field when present (same result as a bare SignedDistanceField).
 *   - With no field attached, SDFMesh behaves exactly like an equivalent Mesh on Bullet and FCL.
 *   - FCL always uses the triangle surface, ignoring any attached field.
 *   - The continuous/cast manager accepts a field-less SDFMesh (a mesh) but rejects one with a
 *     concave field attached.
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

using namespace tesseract::collision;
namespace tg = tesseract::geometry;

namespace
{
using VertsFaces = std::pair<std::shared_ptr<tesseract::common::VectorVector3d>, std::shared_ptr<Eigen::VectorXi>>;

/** @brief A discretized signed distance field for a sphere of radius 0.5 centred at the origin. */
tg::SignedDistanceField::Ptr makeSphereSDFField()
{
  const tg::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  return tg::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(16, 16, 16));
}

/**
 * @brief A small two-triangle surface. The first triangle lies in the z = 0 plane (so the probe
 * sphere below contacts it predictably); the fourth vertex is lifted so the point set is
 * non-coplanar (FCL's OBBRSS fit is degenerate for perfectly planar input).
 */
VertsFaces makeQuad()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(-1, 1, 0.5);  // lifted -> non-coplanar

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces) << 3, 0, 1, 2, 3, 0, 2, 3;
  return { vertices, faces };
}

/** @brief An SDFMesh carrying a (minimal) triangle surface plus the discretized sphere field. */
CollisionShapePtr makeSphereSDFMesh()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(0, 0, 0);
  vertices->emplace_back(1, 0, 0);
  vertices->emplace_back(0, 1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(4);
  (*faces) << 3, 0, 1, 2;

  auto sdf_mesh = std::make_shared<tg::SDFMesh>(vertices, faces);
  sdf_mesh->setSignedDistanceField(makeSphereSDFField());
  return sdf_mesh;
}

/**
 * @brief Drop @p surface and a probe sphere (r = 0.1) just above a z = 0 surface into @p checker and
 * return the closest contact distance (NaN if there is no contact).
 *
 * The probe is positioned over the interior of the first (z = 0) triangle with a small positive
 * gap: its centre sits at z = 0.13, so the sphere surface is 0.03 above the plane, inside the 0.05
 * collision margin. A flat quad therefore reports a separation distance of +0.03.
 *
 * @note The configuration is intentionally separated (positive distance) rather than penetrating.
 * FCL only supports negative/penetration distance for primitive ("OT_GEOM") shapes; for a triangle
 * mesh (BVHModel) it falls back to a collision-based workaround that aborts with
 * `assert(index != -1)` when collide() finds no contact for a thin sheet. Keeping the probe
 * separated exercises FCL's normal mesh-distance path and stays valid for Bullet as well.
 */
double probeDistance(DiscreteContactManager& checker, const CollisionShapePtr& surface)
{
  CollisionShapesConst surf_shapes{ surface };
  tesseract::common::VectorIsometry3d surf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("surface", 0, surf_shapes, surf_poses, true);

  CollisionShapesConst probe_shapes{ std::make_shared<tg::Sphere>(0.1) };
  tesseract::common::VectorIsometry3d probe_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("probe", 0, probe_shapes, probe_poses, true);

  checker.setActiveCollisionObjects({ "surface", "probe" });
  checker.setDefaultCollisionMargin(0.05);

  tesseract::common::TransformMap location;
  location["surface"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d probe_tf = Eigen::Isometry3d::Identity();
  probe_tf.translation() = Eigen::Vector3d(0.33, -0.33, 0.13);  // over the interior of the z = 0 triangle
  location["probe"] = probe_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);
  return result_vector.empty() ? std::numeric_limits<double>::quiet_NaN() : result_vector.front().distance;
}

// An SDFMesh with a field attached collides via the field, matching a bare SignedDistanceField:
// SDF(0.45,0,0) - sphere_radius = -0.05 - 0.1 = -0.15.
void runSDFMeshFieldTest(DiscreteContactManager& checker)
{
  CollisionShapesConst sdf_shapes{ makeSphereSDFMesh() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true);

  CollisionShapesConst sphere_shapes{ std::make_shared<tg::Sphere>(0.1) };
  tesseract::common::VectorIsometry3d sphere_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sphere_link", 0, sphere_shapes, sphere_poses, true);

  checker.setActiveCollisionObjects({ "sdf_link", "sphere_link" });
  checker.setDefaultCollisionMargin(0.0);

  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
  sphere_tf.translation().x() = 0.45;
  location["sphere_link"] = sphere_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_LT(result_vector.front().distance, 0.0);
  EXPECT_NEAR(result_vector.front().distance, -0.15, 0.02);
}
}  // namespace

TEST(TesseractCollisionSDFMeshUnit, BulletDiscreteSimpleUsesAttachedField)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFMeshFieldTest(checker);
}

TEST(TesseractCollisionSDFMeshUnit, BulletDiscreteBVHUsesAttachedField)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFMeshFieldTest(checker);
}

TEST(TesseractCollisionSDFMeshUnit, BulletFieldlessSDFMeshMatchesMesh)  // NOLINT
{
  auto [vertices, faces] = makeQuad();
  BulletDiscreteBVHManager mesh_checker;
  const double d_mesh = probeDistance(mesh_checker, std::make_shared<tg::Mesh>(vertices, faces));
  BulletDiscreteBVHManager sdf_checker;
  const double d_sdf = probeDistance(sdf_checker, std::make_shared<tg::SDFMesh>(vertices, faces));

  ASSERT_FALSE(std::isnan(d_mesh));
  ASSERT_FALSE(std::isnan(d_sdf));
  EXPECT_NEAR(d_mesh, d_sdf, 1e-9);
}

TEST(TesseractCollisionSDFMeshUnit, FCLFieldlessSDFMeshMatchesMesh)  // NOLINT
{
  auto [vertices, faces] = makeQuad();
  FCLDiscreteBVHManager mesh_checker;
  const double d_mesh = probeDistance(mesh_checker, std::make_shared<tg::Mesh>(vertices, faces));
  FCLDiscreteBVHManager sdf_checker;
  const double d_sdf = probeDistance(sdf_checker, std::make_shared<tg::SDFMesh>(vertices, faces));

  ASSERT_FALSE(std::isnan(d_mesh));
  ASSERT_FALSE(std::isnan(d_sdf));
  EXPECT_NEAR(d_mesh, d_sdf, 1e-9);
}

TEST(TesseractCollisionSDFMeshUnit, FCLIgnoresAttachedFieldUsesMesh)  // NOLINT
{
  // FCL cannot consume a distance field; an SDFMesh with one attached must still collide via its
  // triangle surface, matching the equivalent Mesh.
  auto [vertices, faces] = makeQuad();
  FCLDiscreteBVHManager mesh_checker;
  const double d_mesh = probeDistance(mesh_checker, std::make_shared<tg::Mesh>(vertices, faces));

  auto sdf_mesh = std::make_shared<tg::SDFMesh>(vertices, faces);
  sdf_mesh->setSignedDistanceField(makeSphereSDFField());
  FCLDiscreteBVHManager sdf_checker;
  const double d_sdf = probeDistance(sdf_checker, sdf_mesh);

  ASSERT_FALSE(std::isnan(d_mesh));
  ASSERT_FALSE(std::isnan(d_sdf));
  EXPECT_NEAR(d_mesh, d_sdf, 1e-9);
}

TEST(TesseractCollisionSDFMeshUnit, CastManagerAcceptsFieldlessSDFMesh)  // NOLINT
{
  // A field-less SDFMesh is a triangle mesh, so the continuous/cast manager can convexify it.
  auto [vertices, faces] = makeQuad();
  BulletCastBVHManager checker;
  CollisionShapesConst shapes{ std::make_shared<tg::SDFMesh>(vertices, faces) };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  EXPECT_NO_THROW(checker.addCollisionObject("sdf_mesh_link", 0, shapes, poses, true));  // NOLINT
}

TEST(TesseractCollisionSDFMeshUnit, CastManagerRejectsSDFMeshWithField)  // NOLINT
{
  // With a (concave) field attached the cast manager must reject it, as for a bare SignedDistanceField.
  BulletCastBVHManager checker;
  CollisionShapesConst shapes{ makeSphereSDFMesh() };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  EXPECT_ANY_THROW(checker.addCollisionObject("sdf_mesh_link", 0, shapes, poses, true));  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
