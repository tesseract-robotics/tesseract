#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/test_suite/collision_box_box_cast_unit.hpp>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionBoxBoxUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionBoxBoxUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
