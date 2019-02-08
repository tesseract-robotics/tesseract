#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

void addCollisionObjects(tesseract::DiscreteContactManager& checker)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  tesseract::CollisionShapePtr box(new tesseract::BoxCollisionShape(1, 1, 1));
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  tesseract::CollisionShapesConst obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  tesseract::CollisionShapePtr thin_box(new tesseract::BoxCollisionShape(0.1, 1, 1));
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  tesseract::CollisionShapesConst obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

  /////////////////////////////////////////////////////////////////
  // Add cone to checker. If use_convex_mesh = true then this
  // cone will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  tesseract::CollisionShapePtr cone(new tesseract::ConeCollisionShape(0.25, 0.25));
  Eigen::Isometry3d cone_pose;
  cone_pose.setIdentity();

  tesseract::CollisionShapesConst obj3_shapes;
  tesseract::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(cone);
  obj3_poses.push_back(cone_pose);

  checker.addCollisionObject("cone_link", 0, obj3_shapes, obj3_poses);
}

void runTest(tesseract::DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "cone_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["cone_link"] = Eigen::Isometry3d::Identity();
  location["cone_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

  tesseract::ContactResultVector result_vector;
  tesseract::flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.55, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], -0.125, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], -0.125, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["cone_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();

  checker.setCollisionObjectsTransform(location);
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.251);
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.25, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], -0.125, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], -0.125, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxConeUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxConeUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxConeUnit)
{
  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
