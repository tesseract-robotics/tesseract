/**
 * @file collision_signed_distance_field_unit.cpp
 * @brief Discrete collision tests for the volumetric SignedDistanceField geometry against a sphere.
 *
 * A signed distance field for a sphere of radius R centered at the origin is discretized at runtime
 * into the Discregrid / btMiniSDF binary blob format and fed to the Bullet backend. A probe
 * sphere is then placed at known locations and the reported contact distance is validated. This
 * exercises both the SDF data-ingestion path (initializeSDF) and the convex-vs-SDF narrowphase.
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
#include <cstdint>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/bullet_collision_shape_cache.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

using namespace tesseract::collision;

namespace
{
CollisionShapePtr makeSphereSDF()
{
  // Discretize the field with the production sampler (shared tricubic nodes), exercising the real
  // code path end-to-end: f(p) = ||p|| - 0.5 over [-1, 1]^3 at 16 cells per axis.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  return tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(16, 16, 16));
}

void addObjects(DiscreteContactManager& checker)
{
  // SDF link (concave sphere distance field)
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true);

  // Probe sphere
  CollisionShapesConst sphere_shapes{ std::make_shared<tesseract::geometry::Sphere>(0.1) };
  tesseract::common::VectorIsometry3d sphere_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sphere_link", 0, sphere_shapes, sphere_poses, true);

  checker.setActiveCollisionObjects({ "sdf_link", "sphere_link" });
  checker.setDefaultCollisionMargin(0.0);
}

void runSDFTest(DiscreteContactManager& checker)
{
  addObjects(checker);

  // Place the probe sphere center just inside the SDF surface: SDF(0.45,0,0) = 0.45 - 0.5 = -0.05.
  // Reported contact distance = SDF(center) - sphere_radius = -0.05 - 0.1 = -0.15.
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
  // The penetration distance should be negative and close to the analytic value.
  EXPECT_LT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, -0.15, 0.02);

  // The SDF gradient must be surfaced as the contact normal -- trajopt's collision Jacobian
  // (J^T * normal) depends on it. For a sphere field probed along +x the gradient is radial, so
  // the normal must be populated, unit length, and aligned with the x-axis.
  const Eigen::Vector3d& normal = result_vector[0].normal;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 0.05);
  EXPECT_NEAR(normal.y(), 0.0, 0.05);
  EXPECT_NEAR(normal.z(), 0.0, 0.05);

  // Move the probe sphere well outside the SDF surface -> no contact at zero margin.
  sphere_tf.translation().x() = 0.9;  // SDF here = 0.4, far beyond the sphere radius
  checker.setCollisionObjectsTransform("sphere_link", sphere_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}

void runSDFPositiveDistanceTest(DiscreteContactManager& checker)
{
  // POSITIVE-distance (separation) contacts are the load-bearing case for motion
  // planning: TrajOpt's collision costs, contact-check gates, and OMPL admission
  // margins all consume contacts reported at distance > 0 within the configured
  // margin band -- never just boolean penetration. A backend that only reports
  // penetrating contacts silently breaks every margin-based consumer.
  addObjects(checker);
  checker.setDefaultCollisionMargin(0.05);  // report contacts out to 5cm separation

  // Probe center at x = 0.63: SDF(center) = 0.63 - 0.5 = 0.13; separation to the
  // probe sphere surface = 0.13 - 0.1 = +0.03, INSIDE the 0.05 margin band.
  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
  sphere_tf.translation().x() = 0.63;
  location["sphere_link"] = sphere_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_GT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, 0.03, 0.02);

  // Gradient normal must be surfaced for separated contacts too (trajopt pushes
  // AWAY along it before penetration ever happens).
  const Eigen::Vector3d& normal = result_vector[0].normal;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 0.05);

  // Separation OUTSIDE the margin band: at x = 0.75 the SDF at the probe center
  // is 0.25, so separation = 0.25 - 0.1 = +0.15 > the 0.05 margin -> no contact.
  sphere_tf.translation().x() = 0.75;
  checker.setCollisionObjectsTransform("sphere_link", sphere_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}
}  // namespace

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimple)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVH)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimplePositiveDistance)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFPositiveDistanceTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVHPositiveDistance)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFPositiveDistanceTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, CastManagerRejects)  // NOLINT
{
  // The cast/continuous manager only supports convex shapes; an SDF (concave) must be rejected
  // with a clear error rather than silently misbehaving.
  BulletCastBVHManager checker;
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  EXPECT_ANY_THROW(checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true));  // NOLINT
}

TEST(TesseractCollisionSignedDistanceFieldUnit, CreateShapePrimitiveBranches)  // NOLINT
{
  using tesseract::collision::bullet_internal::createShapePrimitive;

  // Empty grid (default-constructed) -> defensive nullptr (no shape created).
  {
    auto geom = std::make_shared<tesseract::geometry::SignedDistanceField>();
    EXPECT_EQ(createShapePrimitive(geom), nullptr);
  }

  // Valid field with a positive margin -> non-null shape with that margin applied.
  {
    const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
    auto geom = tesseract::geometry::createDiscreteSignedDistanceField(sphere,
                                                                       Eigen::Vector3d(-1, -1, -1),
                                                                       Eigen::Vector3d(1, 1, 1),
                                                                       Eigen::Vector3i(8, 8, 8),
                                                                       Eigen::Vector3d(1, 1, 1),
                                                                       0.05);
    auto shape = createShapePrimitive(geom);
    ASSERT_NE(shape, nullptr);
    ASSERT_NE(shape->top_level, nullptr);
    EXPECT_NEAR(static_cast<double>(shape->top_level->getMargin()), 0.05, 1e-5);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
