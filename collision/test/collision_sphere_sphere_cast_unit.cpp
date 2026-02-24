#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_sphere_sphere_cast_unit.hpp>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionSphereSphereUnit)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionSphereSphereUnit)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTest(checker, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
