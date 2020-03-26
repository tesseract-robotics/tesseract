#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_octomap_sphere_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, 0.001, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, 0.02, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.001, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.02, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.16, false);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.16, true);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
