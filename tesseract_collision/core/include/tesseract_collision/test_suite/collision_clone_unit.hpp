#ifndef TESSERACT_COLLISION_COLLISION_CLONE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CLONE_UNIT_HPP

#include <tesseract_collision/core/discrete_contact_manager.h>
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

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

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
  EXPECT_TRUE(checker.isCollisionObjectEnabled("sphere1_link"));

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
  EXPECT_TRUE(checker.isCollisionObjectEnabled("remove_box_link"));
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.isCollisionObjectEnabled("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract_common::VectorIsometry3d()));

  /////////////////////////////////////////////
  // Check sizes
  /////////////////////////////////////////////
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
  for (const auto& co : checker.getCollisionObjects())
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co).size() == 1);
    for (const auto& cgt : checker.getCollisionObjectGeometriesTransforms(co))
    {
      EXPECT_TRUE(cgt.isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
  }
}
}  // namespace detail

inline void
runTest(DiscreteContactManager& checker, double dist_tol = 0.001, double nearest_tol = 0.001, double normal_tol = 0.001)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

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
  flattenResults(std::move(result), result_vector);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  // Clone and perform collision check
  ContactResultMap cloned_result;
  DiscreteContactManager::Ptr cloned_checker = checker.clone();

  cloned_checker->contactTest(cloned_result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector cloned_result_vector;
  flattenResults(std::move(cloned_result), cloned_result_vector);

  std::vector<int> cloned_idx = { 0, 1, 1 };
  if (cloned_result_vector[0].link_names[0] != "sphere_link")
    cloned_idx = { 1, 0, -1 };

  EXPECT_FALSE(cloned_checker->isCollisionObjectEnabled("thin_box_link"));
  EXPECT_TRUE(!result_vector.empty() && !cloned_result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, cloned_result_vector[0].distance, dist_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[0])][0],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[0])][1],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[0])][2],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[1])][0],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[1])][1],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2],
              cloned_result_vector[0].nearest_points[static_cast<size_t>(cloned_idx[1])][2],
              nearest_tol);
  EXPECT_NEAR(result_vector[0].normal[0] * idx[2], cloned_result_vector[0].normal[0] * cloned_idx[2], normal_tol);
  EXPECT_NEAR(result_vector[0].normal[1] * idx[2], cloned_result_vector[0].normal[1] * cloned_idx[2], normal_tol);
  EXPECT_NEAR(result_vector[0].normal[2] * idx[2], cloned_result_vector[0].normal[2] * cloned_idx[2], normal_tol);
}
}  // namespace tesseract_collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_CLONE_UNIT_HPP
