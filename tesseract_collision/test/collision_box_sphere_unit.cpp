#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

void addCollisionObjects(tesseract::DiscreteContactManagerBase& checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr sphere;
  if (use_convex_mesh)
    sphere.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  tesseract::VectorIsometry3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);

  if (use_convex_mesh)
    obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere_link", 0, obj3_shapes, obj3_poses, obj3_types);
}

void runTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "sphere_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.55, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.251);
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.25, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

void runConvexTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "sphere_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.53776, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.03776, 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.27);
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.26224, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.76224, 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereConvexHullUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereConvexHullUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxSphereUnit)
{
  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

// TODO: Levi, enable once FCL PR #338
// TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxSphereConvexHullUnit)
//{
//  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
//  addCollisionObjects(checker, true);
//  runConvexTest(checker);
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
