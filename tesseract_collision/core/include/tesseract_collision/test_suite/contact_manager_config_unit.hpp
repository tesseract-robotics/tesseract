#ifndef TESSERACT_COLLISION_CONTACT_MANAGER_CONFIG_UNIT_H
#define TESSERACT_COLLISION_CONTACT_MANAGER_CONFIG_UNIT_H

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  CollisionShapePtr sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses);

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

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1 = std::make_shared<tesseract_geometry::Sphere>(0.25);

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);
}

inline void addCollisionObjects(ContinuousContactManager& checker)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  CollisionShapePtr sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation()[2] = 0.25;

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses, false);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("sphere_link"));
  checker.enableCollisionObject("sphere_link");
  EXPECT_TRUE(checker.isCollisionObjectEnabled("sphere_link"));

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

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1 = std::make_shared<tesseract_geometry::Sphere>(0.25);

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();
  sphere1_pose.translation()[2] = 0.25;

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);
  EXPECT_TRUE(checker.isCollisionObjectEnabled("sphere1_link"));
}

}  // namespace detail

inline void runTest(DiscreteContactManager& checker)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  std::vector<std::string> active_links{ "sphere_link", "sphere1_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract_common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getIsContactAllowedFn() == nullptr);

  checker.setDefaultCollisionMarginData(0.5);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.5, 1e-5);
  EXPECT_TRUE(checker.isCollisionObjectEnabled("thin_box_link"));

  ContactManagerConfig config(0.1);
  config.margin_data_override_type = CollisionMarginOverrideType::NONE;
  config.modify_object_enabled["thin_box_link"] = false;

  checker.applyContactManagerConfig(config);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.5, 1e-5);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

  config.margin_data_override_type = CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  checker.applyContactManagerConfig(config);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Test when object is inside another
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.30, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.0016);  // TODO LEVI: This was increased due to FLC
                                                                  // calculation because it is using GJK

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform("sphere1_link", location["sphere1_link"]);

  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenCopyResults(result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();

  config = ContactManagerConfig(0.52);
  checker.applyContactManagerConfig(config);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.52, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

inline void runTest(ContinuousContactManager& checker)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  ///////////////////////////////////////////////////
  std::vector<std::string> active_links{ "sphere_link", "sphere1_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract_common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getIsContactAllowedFn() == nullptr);

  checker.setDefaultCollisionMarginData(0.5);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.5, 1e-5);
  EXPECT_TRUE(checker.isCollisionObjectEnabled("thin_box_link"));

  ContactManagerConfig config(0.1);
  config.margin_data_override_type = CollisionMarginOverrideType::NONE;
  config.modify_object_enabled["thin_box_link"] = false;

  checker.applyContactManagerConfig(config);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.5, 1e-5);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

  config.margin_data_override_type = CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  checker.applyContactManagerConfig(config);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the start location
  tesseract_common::TransformMap location_start;
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -1.0;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  tesseract_common::TransformMap location_end;
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenCopyResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.1, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[0])], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][0], -0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(
      result_vector[0].cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001));

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  /////////////////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.333 and 0.5
  /////////////////////////////////////////////////////////////

  // Set the start location
  location_start["sphere_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere_link"].translation()(0) = -0.2;
  location_start["sphere_link"].translation()(1) = -0.5;

  location_start["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_start["sphere1_link"].translation()(0) = 0.2;
  location_start["sphere1_link"].translation()(2) = -1.0;

  // Set the end location
  location_end["sphere_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere_link"].translation()(0) = -0.2;
  location_end["sphere_link"].translation()(1) = 1.0;

  location_end["sphere1_link"] = Eigen::Isometry3d::Identity();
  location_end["sphere1_link"].translation()(0) = 0.2;
  location_end["sphere1_link"].translation()(2) = 1.0;

  checker.setCollisionObjectsTransform(location_start, location_end);

  // Perform collision check
  result.clear();
  result_vector.clear();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.1, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[0])], 0.3333, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[1]))] ==
              ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][0], -0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[static_cast<size_t>(idx[0])].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[static_cast<size_t>(idx[1])].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[static_cast<size_t>(idx[0])].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(
      result_vector[0].cc_transform[static_cast<size_t>(idx[1])].isApprox(location_end["sphere1_link"], 0.0001));

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}
}  // namespace tesseract_collision::test_suite
#endif  // TESSERACT_COLLISION_CONTACT_MANAGER_CONFIG_UNIT_H
