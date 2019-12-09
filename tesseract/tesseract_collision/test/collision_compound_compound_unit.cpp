#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

template <class T>
void addCollisionObjects(T& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(DATA_DIR) + "/box_2m.bt";
  std::shared_ptr<octomap::OcTree> ot(new octomap::OcTree(path));
  CollisionShapePtr dense_octomap(new Octree(ot, Octree::BOX));
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();
  octomap_pose.translation() = Eigen::Vector3d(1.1, 0, 0);

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap1_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  std::shared_ptr<octomap::OcTree> ot_b(new octomap::OcTree(path));
  CollisionShapePtr dense_octomap_b(new Octree(ot_b, Octree::BOX));
  Eigen::Isometry3d octomap_pose_b;
  octomap_pose_b.setIdentity();
  octomap_pose_b.translation() = Eigen::Vector3d(-1.1, 0, 0);

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(dense_octomap_b);
  obj2_poses.push_back(octomap_pose_b);

  checker.addCollisionObject("octomap2_link", 0, obj2_shapes, obj2_poses);
}

void runTest(DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap1_link", "octomap2_link" });
  checker.setContactDistanceThreshold(0.25);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap1_link"] = Eigen::Isometry3d::Identity();
  location["octomap2_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::ALL);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}

void runCastTest(ContinuousContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap1_link" });
  checker.setContactDistanceThreshold(0.25);

  // Set the collision object transforms
  Eigen::Isometry3d start_pos, end_pos;
  start_pos = Eigen::Isometry3d::Identity();
  end_pos = Eigen::Isometry3d::Identity();
  start_pos.translation() = Eigen::Vector3d(0, -2.0, 0);
  end_pos.translation() = Eigen::Vector3d(0, 2.0, 0);
  checker.setCollisionObjectsTransform("octomap1_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::ALL);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects<DiscreteContactManager>(checker);
  runTest(checker);

  DiscreteContactManager::Ptr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects<DiscreteContactManager>(checker);
  runTest(checker);

  DiscreteContactManager::Ptr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects<DiscreteContactManager>(checker);
  runTest(checker);

  DiscreteContactManager::Ptr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  addCollisionObjects<ContinuousContactManager>(checker);
  runCastTest(checker);

  ContinuousContactManager::Ptr cloned_checker = checker.clone();
  runCastTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionCompoundCompoundUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  addCollisionObjects<ContinuousContactManager>(checker);
  runCastTest(checker);

  ContinuousContactManager::Ptr cloned_checker = checker.clone();
  runCastTest(*cloned_checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
