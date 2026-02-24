#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_sphere_sphere_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
