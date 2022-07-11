#ifndef TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(ContinuousContactManager& checker, bool use_convex_mesh = false)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  CollisionShapePtr sphere;
  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(
        loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces),
        0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_vertices, *ch_faces, mesh_vertices);
    sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
  }

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
  EXPECT_TRUE(checker.isCollisionObjectEnabled("thin_box_link"));
  checker.disableCollisionObject("thin_box_link");
  EXPECT_FALSE(checker.isCollisionObjectEnabled("thin_box_link"));

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(
        loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces),
        0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere1 = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere1 = std::make_shared<tesseract_geometry::Sphere>(0.25);
  }

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();
  sphere1_pose.translation()[2] = 0.25;

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
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);

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
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
}

inline void runTestPrimitive(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  ///////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
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
  flattenResults(std::move(result), result_vector);

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
  result = ContactResultMap();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  result_vector = ContactResultVector();
  flattenResults(std::move(result), result_vector);

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

inline void runTestConvex(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  ///////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
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
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.0754, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[0])], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][0], 0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][0], -0.2377, 0.001);
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
  result = ContactResultMap();
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  result_vector = ContactResultVector();
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.0754, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[0])], 0.3848, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[static_cast<size_t>(idx[1])], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[0]))] ==
              ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(static_cast<size_t>(idx[1]))] ==
              ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0772, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0772, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][0], 0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[0])][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[static_cast<size_t>(idx[1])][0], -0.2377, 0.001);
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
}  // namespace detail

inline void runTest(ContinuousContactManager& checker, bool use_convex_mesh)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  if (use_convex_mesh)
    detail::runTestConvex(checker);
  else
    detail::runTestPrimitive(checker);
}

}  // namespace tesseract_collision::test_suite
#endif  // TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_CAST_UNIT_HPP
