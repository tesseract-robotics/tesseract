#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_sphere_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, 0.001, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker, 0.02, true);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.001, false);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.02, true);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.16, false);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker, 0.16, true);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
