#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_box_sphere_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxSphereUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
