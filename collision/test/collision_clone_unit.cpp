#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_clone_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/geometry/impl/box.h>

using namespace tesseract::collision;
using namespace tesseract::collision::bullet_internal;

namespace
{
/** @brief Build a unit box collision object at the origin, as the managers do */
COW::Ptr makeBoxCow()
{
  CollisionShapesConst shapes{ std::make_shared<tesseract::geometry::Box>(1, 1, 1) };
  tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  return createCollisionObject("box_link", 0, shapes, poses);
}
}  // namespace

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCloneUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, 0.001, 0.001, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCloneUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.001, 0.001, 0.001);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCloneUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.001, 0.001, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCloneOrderUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runCloneOrderTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCloneOrderUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runCloneOrderTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCloneOrderUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runCloneOrderTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionCloneOrderUnit)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runCloneOrderTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionCloneOrderUnit)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runCloneOrderTest(checker);
}

/**
 * The contact processing threshold is what getAABB() pads the broadphase box with, so a clone that
 * loses it enters the broadphase with a box sized by btCollisionObject's BT_LARGE_FLOAT default.
 */
TEST(TesseractCollisionUnit, BulletCollisionObjectWrapperCloneThresholdUnit)  // NOLINT
{
  const btScalar margin{ 0.05 };

  COW::Ptr cow = makeBoxCow();
  cow->setContactProcessingThreshold(margin);

  const COW::Ptr clone = cow->clone();

  EXPECT_NEAR(clone->getContactProcessingThreshold(), margin, 1e-6);
}

/**
 * The cast object is built by cloning, and it is the one whose proxy goes into the broadphase for
 * every active link. With an identity cast transform its AABB must match the object it was built
 * from - CastHullShape::getAabb unions the shape at t0 and t01, which for an identity t01 is just
 * the shape's own AABB.
 */
TEST(TesseractCollisionUnit, BulletCastCollisionObjectCloneAABBUnit)  // NOLINT
{
  const btScalar margin{ 0.05 };

  COW::Ptr cow = makeBoxCow();
  cow->setContactProcessingThreshold(margin);

  const COW::Ptr cast_cow = makeCastCollisionObject(cow);

  btVector3 aabb_min;
  btVector3 aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  btVector3 cast_aabb_min;
  btVector3 cast_aabb_max;
  cast_cow->getAABB(cast_aabb_min, cast_aabb_max);

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(cast_aabb_min[i], aabb_min[i], 1e-6) << "axis " << i;
    EXPECT_NEAR(cast_aabb_max[i], aabb_max[i], 1e-6) << "axis " << i;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
