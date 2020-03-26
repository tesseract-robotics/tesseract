#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_multi_threaded_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

using namespace tesseract_collision;

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
