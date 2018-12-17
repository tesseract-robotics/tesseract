#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

void addCollisionObjects(tesseract::DiscreteContactManagerBase& checker)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  shapes::ShapePtr sphere;
  sphere.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses, obj1_types);

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
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr sphere1;
  sphere1.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  tesseract::VectorIsometry3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);
  obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses, obj3_types);
}

void runTest(tesseract::DiscreteContactManagerBase& checker, double dist_tol, double nearest_tol, double normal_tol)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0.1);

  // Test when object is inside another
  tesseract::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  // Clone and perform collision check
  tesseract::ContactResultMap cloned_result;
  tesseract::DiscreteContactManagerBasePtr cloned_checker = checker.clone();

  cloned_checker->contactTest(cloned_result, tesseract::ContactTestType::CLOSEST);

  tesseract::ContactResultVector cloned_result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(cloned_result, cloned_result_vector);

  std::vector<int> cloned_idx = { 0, 1, 1 };
  if (cloned_result_vector[0].link_names[0] != "sphere_link")
    cloned_idx = { 1, 0, -1 };

  EXPECT_TRUE(!result_vector.empty() && !cloned_result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, cloned_result_vector[0].distance, dist_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0],
              cloned_result_vector[0].nearest_points[cloned_idx[0]][0],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1],
              cloned_result_vector[0].nearest_points[cloned_idx[0]][1],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2],
              cloned_result_vector[0].nearest_points[cloned_idx[0]][2],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0],
              cloned_result_vector[0].nearest_points[cloned_idx[1]][0],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1],
              cloned_result_vector[0].nearest_points[cloned_idx[1]][1],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2],
              cloned_result_vector[0].nearest_points[cloned_idx[1]][2],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].normal[0] * idx[2], cloned_result_vector[0].normal[0] * cloned_idx[2], normal_tol);
  EXPECT_NEAR(result_vector[0].normal[1] * idx[2], cloned_result_vector[0].normal[1] * cloned_idx[2], normal_tol);
  EXPECT_NEAR(result_vector[0].normal[2] * idx[2], cloned_result_vector[0].normal[2] * cloned_idx[2], normal_tol);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionCloneUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001, 0.001, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionCloneUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001, 0.001, 0.001);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionCloneUnit)
{
  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001, 0.001, 0.005);  // TODO: FCL requires a large tolerance because it using GJK currently
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
