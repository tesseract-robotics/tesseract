#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

template <class T>
void addCollisionObjects(T& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = ros::package::getPath("tesseract_collision") + "/test/box_2m.bt";
  std::shared_ptr<octomap::OcTree> ot(new octomap::OcTree(path));
  shapes::ShapePtr dense_octomap(new shapes::OcTree(ot));
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();
  octomap_pose.translation() = Eigen::Vector3d(1.1, 0, 0);

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("octomap1_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  std::shared_ptr<octomap::OcTree> ot_b(new octomap::OcTree(path));
  shapes::ShapePtr dense_octomap_b(new shapes::OcTree(ot_b));
  Eigen::Isometry3d octomap_pose_b;
  octomap_pose_b.setIdentity();
  octomap_pose_b.translation() = Eigen::Vector3d(-1.1, 0, 0);

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(dense_octomap_b);
  obj2_poses.push_back(octomap_pose_b);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("octomap2_link", 0, obj2_shapes, obj2_poses, obj2_types);
}

void runTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap1_link", "octomap2_link" });
  checker.setContactDistanceThreshold(0.25);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["octomap1_link"] = Eigen::Isometry3d::Identity();
  location["octomap2_link"] = Eigen::Isometry3d::Identity();
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::ALL);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}

void runCastTest(tesseract::ContinuousContactManagerBase& checker)
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
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::ALL);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  for (const auto& cr : result_vector)
  {
    EXPECT_NEAR(cr.distance, 0.20, 0.001);
  }
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCompoundCompoundUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker);
  runTest(checker);

  tesseract::DiscreteContactManagerBasePtr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCompoundCompoundUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker);
  runTest(checker);

  tesseract::DiscreteContactManagerBasePtr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCompoundCompoundUnit)
{
  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker);
  runTest(checker);

  tesseract::DiscreteContactManagerBasePtr cloned_checker = checker.clone();
  runTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionCompoundCompoundUnit)
{
  tesseract::tesseract_bullet::BulletCastSimpleManager checker;
  addCollisionObjects<tesseract::ContinuousContactManagerBase>(checker);
  runCastTest(checker);

  tesseract::ContinuousContactManagerBasePtr cloned_checker = checker.clone();
  runCastTest(*cloned_checker);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionCompoundCompoundUnit)
{
  tesseract::tesseract_bullet::BulletCastBVHManager checker;
  addCollisionObjects<tesseract::ContinuousContactManagerBase>(checker);
  runCastTest(checker);

  tesseract::ContinuousContactManagerBasePtr cloned_checker = checker.clone();
  runCastTest(*cloned_checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
