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
#include <stdexcept>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/bullet_collision_shape_cache.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
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

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVH)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHPositiveDistance)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFPositiveDistanceTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHSDFPair)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  CollisionShapesConst sdf0_shapes{ makeSphereSDF() };
  CollisionShapesConst sdf1_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf0_link", 0, sdf0_shapes, poses, true);
  checker.addCollisionObject("sdf1_link", 0, sdf1_shapes, poses, true);
  checker.setActiveCollisionObjects({ "sdf0_link", "sdf1_link" });

  tesseract::common::TransformMap location;
  location["sdf0_link"] = Eigen::Isometry3d::Identity();
  location["sdf1_link"] = Eigen::Translation3d(0.8, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_NEAR(result_vector.front().distance, -0.2, 0.02);
  EXPECT_NEAR(result_vector.front().normal.norm(), 1.0, 1e-3);

  checker.setCollisionMarginPair("sdf0_link", "sdf1_link", 0.05);
  location["sdf1_link"] = Eigen::Translation3d(1.03, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);
  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  ASSERT_FALSE(result_vector.empty());
  EXPECT_NEAR(result_vector.front().distance, 0.03, 0.02);

  location["sdf1_link"] = Eigen::Translation3d(1.06, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);
  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLRejectsUnsupportedSDFPair)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  CollisionShapesConst plane_shapes{ std::make_shared<tesseract::geometry::Plane>(0.0, 0.0, 1.0, 0.0) };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, poses, true);
  checker.addCollisionObject("plane_link", 0, plane_shapes, poses, true);
  checker.setActiveCollisionObjects({ "sdf_link", "plane_link" });

  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  location["plane_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  EXPECT_THROW(checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST)), std::runtime_error);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLRejectsNonuniformSDFScale)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  const Eigen::AlignedBox3d domain(Eigen::Vector3d::Constant(-1.0), Eigen::Vector3d::Constant(1.0));
  auto sdf = std::make_shared<tesseract::geometry::SignedDistanceField>(
      domain, Eigen::Vector3i(2, 2, 2), std::vector<double>(8, 0.0), Eigen::Vector3d(1.0, 2.0, 1.0));
  CollisionShapesConst sdf_shapes{ sdf };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  EXPECT_THROW(checker.addCollisionObject("sdf_link", 0, sdf_shapes, poses, true), std::invalid_argument);
}

void addCapsuleObjects(DiscreteContactManager& checker)
{
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true);

  // Probe capsule: radius 0.1, cylindrical length 0.2 (bullet axis = z)
  CollisionShapesConst capsule_shapes{ std::make_shared<tesseract::geometry::Capsule>(0.1, 0.2) };
  tesseract::common::VectorIsometry3d capsule_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("capsule_link", 0, capsule_shapes, capsule_poses, true);

  checker.setActiveCollisionObjects({ "sdf_link", "capsule_link" });
  checker.setDefaultCollisionMargin(0.0);
}

void runSDFCapsuleTest(DiscreteContactManager& checker)
{
  // Capsules are checked by sampling the field along the axis segment with a radius offset. The
  // capsule axis is z, so at x = 0.45 the deepest sample is the axis midpoint:
  // SDF(0.45,0,0) - radius = -0.05 - 0.1 = -0.15, same as the sphere probe.
  addCapsuleObjects(checker);

  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsule_tf = Eigen::Isometry3d::Identity();
  capsule_tf.translation().x() = 0.45;
  location["capsule_link"] = capsule_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_LT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, -0.15, 0.02);

  const Eigen::Vector3d& normal = result_vector[0].normal;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 0.05);

  // Positive-distance (separation) contact inside the margin band, mirroring the sphere case:
  // at x = 0.63 the axis midpoint separation is 0.13 - 0.1 = +0.03 < the 0.05 margin.
  checker.setDefaultCollisionMargin(0.05);
  capsule_tf.translation().x() = 0.63;
  checker.setCollisionObjectsTransform("capsule_link", capsule_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_GT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, 0.03, 0.02);

  // Separation outside the margin band -> no contact.
  capsule_tf.translation().x() = 0.75;
  checker.setCollisionObjectsTransform("capsule_link", capsule_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}

void runSDFBoxTest(DiscreteContactManager& checker)
{
  // Exercise the generic implicit solver's polyhedral primitive adapter.
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true);

  // Probe box with side 0.2 (half extents 0.1).
  CollisionShapesConst box_shapes{ std::make_shared<tesseract::geometry::Box>(0.2, 0.2, 0.2) };
  tesseract::common::VectorIsometry3d box_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("box_link", 0, box_shapes, box_poses, true);

  checker.setActiveCollisionObjects({ "sdf_link", "box_link" });
  checker.setDefaultCollisionMargin(0.0);

  // Penetration: the box's inner face is at x = 0.4 and the sphere surface is at x = 0.5,
  // giving a surface-to-surface penetration depth of 0.1.
  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d box_tf = Eigen::Isometry3d::Identity();
  box_tf.translation().x() = 0.5;
  location["box_link"] = box_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_LT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, -0.1, 0.02);

  // The nearest surfaces are the sphere and box faces along the x-axis.
  const Eigen::Vector3d& normal = result_vector[0].normal;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 0.02);

  // Positive-distance contact inside the margin band: box center at x = 0.62 puts its inner face
  // at x = 0.52, giving 0.02 separation from the sphere.
  checker.setDefaultCollisionMargin(0.05);
  box_tf.translation().x() = 0.62;
  checker.setCollisionObjectsTransform("box_link", box_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_GT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].distance, 0.02, 0.02);

  // Separation outside the margin band: box center at x = 0.7 gives 0.1 face separation.
  box_tf.translation().x() = 0.7;
  checker.setCollisionObjectsTransform("box_link", box_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimpleCapsule)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFCapsuleTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVHCapsule)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFCapsuleTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHCapsule)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFCapsuleTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimpleBox)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFBoxTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVHBox)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFBoxTest(checker);
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHBox)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFBoxTest(checker);
}

void runSDFCurvedPrimitiveTest(DiscreteContactManager& checker,
                               const CollisionShapeConstPtr& primitive,
                               const std::string& primitive_link)
{
  CollisionShapesConst sdf_shapes{ makeSphereSDF() };
  tesseract::common::VectorIsometry3d sdf_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject("sdf_link", 0, sdf_shapes, sdf_poses, true);

  CollisionShapesConst primitive_shapes{ primitive };
  tesseract::common::VectorIsometry3d primitive_poses{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject(primitive_link, 0, primitive_shapes, primitive_poses, true);

  checker.setActiveCollisionObjects({ "sdf_link", primitive_link });
  checker.setDefaultCollisionMargin(0.0);

  // Place the primitive across the SDF surface. Sampling the exact barrel/side and cap patches
  // must produce at least one penetrating contact.
  tesseract::common::TransformMap location;
  location["sdf_link"] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d primitive_tf = Eigen::Isometry3d::Identity();
  primitive_tf.translation().x() = 0.45;
  location[primitive_link] = primitive_tf;
  checker.setCollisionObjectsTransform(location);

  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_LT(result_vector[0].distance, 0.0);
  EXPECT_NEAR(result_vector[0].normal.norm(), 1.0, 1e-3);

  // At this pose both primitives are separated from the sphere but remain inside a 5 cm pair
  // margin. This verifies that sampled primitive contacts preserve Tesseract's margin contract.
  checker.setDefaultCollisionMargin(0.05);
  primitive_tf.translation().x() = 0.63;
  checker.setCollisionObjectsTransform(primitive_link, primitive_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  ASSERT_FALSE(result_vector.empty());
  EXPECT_GT(result_vector[0].distance, 0.0);
  EXPECT_LE(result_vector[0].distance, 0.05);

  // Move beyond the primitive extent and pair margin.
  primitive_tf.translation().x() = 0.8;
  checker.setCollisionObjectsTransform(primitive_link, primitive_tf);

  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);
  EXPECT_TRUE(result_vector.empty());
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimpleCylinder)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.2), "cylinder_link");
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVHCylinder)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.2), "cylinder_link");
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHCylinder)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cylinder>(0.1, 0.2), "cylinder_link");
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteSimpleCone)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cone>(0.1, 0.2), "cone_link");
}

TEST(TesseractCollisionSignedDistanceFieldUnit, BulletDiscreteBVHCone)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cone>(0.1, 0.2), "cone_link");
}

TEST(TesseractCollisionSignedDistanceFieldUnit, FCLDiscreteBVHCone)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  runSDFCurvedPrimitiveTest(checker, std::make_shared<tesseract::geometry::Cone>(0.1, 0.2), "cone_link");
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

  // Valid field -> non-null shape.
  {
    const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
    auto geom = tesseract::geometry::createDiscreteSignedDistanceField(sphere,
                                                                       Eigen::Vector3d(-1, -1, -1),
                                                                       Eigen::Vector3d(1, 1, 1),
                                                                       Eigen::Vector3i(8, 8, 8),
                                                                       Eigen::Vector3d(1, 1, 1));
    auto shape = createShapePrimitive(geom);
    ASSERT_NE(shape, nullptr);
    ASSERT_NE(shape->top_level, nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
