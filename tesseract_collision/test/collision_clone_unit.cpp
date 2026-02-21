#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_clone_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

using namespace tesseract::collision;

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
