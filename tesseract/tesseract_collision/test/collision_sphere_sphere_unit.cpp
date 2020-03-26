#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_sphere_sphere_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::detail::addCollisionObjects(checker, true);
  test_suite::detail::runTestConvex1(checker);
  //  test_suite::detail::runTestConvex2(checker); FCL return incorrect results, need to create an issue on FCL
  test_suite::detail::runTestConvex3(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
