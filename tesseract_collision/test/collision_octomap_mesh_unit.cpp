#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_octomap_mesh_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_common/utils.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker,
                      tesseract_common::getTempPath() + "BulletDiscreteSimpleCollisionOctomapSphereMeshUnit.ply");
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker, tesseract_common::getTempPath() + "BulletDiscreteBVHCollisionOctomapSphereMeshUnit.ply");
}

// TODO: There is an issue with fcl octomap collision type. Either with tesseract implementation or fcl
// TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereMeshUnit)
//{
//  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
//  test_suite::runTest(checker, tesseract_common::getTempPath() + "FCLDiscreteBVHCollisionOctomapSphereMeshUnit.ply");
//  // TODO: There appears to be an issue in fcl for octomap::OcTree.
//}

/** @brief This is to test the shape id and subshape id of the contact results. */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
