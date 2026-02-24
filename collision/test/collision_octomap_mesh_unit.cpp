#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/collision_octomap_mesh_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/common.h>
#include <tesseract/common/utils.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereMeshUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker,
                      tesseract::common::getTempPath() + "BulletDiscreteSimpleCollisionOctomapSphereMeshUnit.ply");
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereMeshUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker,
                      tesseract::common::getTempPath() + "BulletDiscreteBVHCollisionOctomapSphereMeshUnit.ply");
}

// TODO: There is an issue with fcl octomap collision type. Either with Tesseract implementation or fcl
// TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereMeshUnit)
//{
//  FCLDiscreteBVHManager checker;
//  test_suite::runTest(checker, tesseract::common::getTempPath() + "FCLDiscreteBVHCollisionOctomapSphereMeshUnit.ply");
//  // TODO: There appears to be an issue in fcl for octomap::OcTree.
//}

/** @brief This is to test the shape id and sub-shape id of the contact results. */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
