#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(ContinuousContactManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr static_box(new Box(1, 1, 1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);

  checker.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses);

  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr moving_box(new Box(0.25, 0.25, 0.25));
  Eigen::Isometry3d moving_box_pose;
  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(moving_box);
  obj2_poses.push_back(moving_box_pose);

  checker.addCollisionObject("moving_box_link", 0, obj2_shapes, obj2_poses);
}

void runTest(ContinuousContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "moving_box_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  checker.setCollisionObjectsTransform("static_box_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation()(0) = -1.9;
  start_pos.translation()(1) = 0.0;
  end_pos.setIdentity();
  end_pos.translation()(0) = 1.9;
  end_pos.translation()(1) = 3.8;
  checker.setCollisionObjectsTransform("moving_box_link", start_pos, end_pos);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.2475, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[0], -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[1], 0.25, 0.001);
  EXPECT_TRUE(result_vector[0].cc_type[0] == ContinuousCollisionType::CCType_None);
  EXPECT_TRUE(result_vector[0].cc_type[1] == ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[0][0], -0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[1][0], -0.325, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][1], 0.325, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][2], 0.0, 0.001);

  Eigen::Vector3d p0 = result_vector[0].transform[1] * result_vector[0].nearest_points_local[1];
  EXPECT_NEAR(p0[0], -1.275, 0.001);
  EXPECT_NEAR(p0[1], -0.625, 0.001);
  EXPECT_NEAR(p0[2], 0.0, 0.001);

  Eigen::Vector3d p1 = result_vector[0].cc_transform[1] * result_vector[0].nearest_points_local[1];
  EXPECT_NEAR(p1[0], 2.525, 0.001);
  EXPECT_NEAR(p1[1], 3.175, 0.001);
  EXPECT_NEAR(p1[2], 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletCastSimpleCollisionBoxBoxUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionBoxBoxUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
