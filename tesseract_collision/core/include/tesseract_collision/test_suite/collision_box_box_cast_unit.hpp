#ifndef TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H
#define TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H

#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(ContinuousContactManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr static_box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);

  checker.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("static_box_link");

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses);
  checker.disableCollisionObject("thin_box_link");

  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  CollisionShapePtr moving_box = std::make_shared<tesseract_geometry::Box>(0.25, 0.25, 0.25);
  Eigen::Isometry3d moving_box_pose;
  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(moving_box);
  obj3_poses.push_back(moving_box_pose);

  checker.addCollisionObject("moving_box_link", 0, obj3_shapes, obj3_poses);

  /////////////////////////////////////////////
  // Add box and remove
  /////////////////////////////////////////////
  CollisionShapePtr remove_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d remove_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj4_shapes;
  tesseract_common::VectorIsometry3d obj4_poses;
  obj4_shapes.push_back(remove_box);
  obj4_poses.push_back(remove_box_pose);

  checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses);
  EXPECT_TRUE(checker.getCollisionObjects().size() == 4);
  EXPECT_TRUE(checker.hasCollisionObject("remove_box_link"));
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract_common::VectorIsometry3d()));

  /////////////////////////////////////////////
  // Check sizes
  /////////////////////////////////////////////
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
  const auto& co = checker.getCollisionObjects();
  for (std::size_t i = 0; i < co.size(); ++i)
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co[i]).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co[i]).size() == 1);
    const auto& cgt = checker.getCollisionObjectGeometriesTransforms(co[i]);
    if (i != 2)
    {
      EXPECT_TRUE(cgt[0].isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
    else
    {
      EXPECT_TRUE(cgt[0].isApprox(moving_box_pose, 1e-5));
    }
  }
}
}  // namespace detail

inline void runTest(ContinuousContactManager& checker)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "moving_box_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the collision object transforms
  std::vector<std::string> names = { "static_box_link" };
  tesseract_common::VectorIsometry3d transforms = { Eigen::Isometry3d::Identity() };
  checker.setCollisionObjectsTransform(names, transforms);

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation()(0) = -1.9;
  start_pos.translation()(1) = 0.0;
  end_pos.setIdentity();
  end_pos.translation()(0) = 1.9;
  end_pos.translation()(1) = 3.8;
  checker.setCollisionObjectsTransform("moving_box_link", start_pos, end_pos);

  std::vector<ContactTestType> test_types = { ContactTestType::ALL, ContactTestType::CLOSEST, ContactTestType::FIRST };

  // Perform collision check
  for (const auto& t : test_types)
  {
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(t));

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
}
}  // namespace tesseract_collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_BOX_BOX_CAST_UNIT_H
