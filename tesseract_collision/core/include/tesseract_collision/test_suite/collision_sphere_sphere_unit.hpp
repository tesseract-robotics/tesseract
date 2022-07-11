#ifndef TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_UNIT_HPP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
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
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
  }

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

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

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

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);

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

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract_common::VectorIsometry3d()));
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
}

inline void runTestPrimitive(DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
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
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.30, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.25)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (-0.05)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][2] - (0.0)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][2] - (0.0)) > 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.05, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.0, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.0016);  // TODO LEVI: This was increased due to FLC
                                                                  // calculation because it is using GJK

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform("sphere1_link", location["sphere1_link"]);

  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setCollisionMarginData(CollisionMarginData(0.52));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.52, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.25)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (0.75)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][2] - (0.0)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][2] - (0.0)) > 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.25, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.75, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.0, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

inline void runTestConvex1(DiscreteContactManager& checker)
{
  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
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
  flattenResults(std::move(result), result_vector);

  // Bullet: -0.270548 {0.232874,0,-0.025368} {-0.032874,0,0.025368}
  // FCL:    -0.270548 {0.232874,0,-0.025368} {-0.032874,0,0.025368}
  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.270548, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.23776)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (-0.032874)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][1] - (0.0)) > 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][2] - (-0.025368)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][2] - (0.025368)) > 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.232874, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], -0.025368, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], -0.032874, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.025368, 0.001);
  }

  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.5);

  ///////////////////////////////////////////////
  // Test object is out side the contact distance
  ///////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());
}

inline void runTestConvex2(DiscreteContactManager& checker)
{
  /////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Edge to Edge)
  /////////////////////////////////////////////////////////////////////////
  ContactResultMap result;
  ContactResultVector result_vector;

  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  checker.setCollisionObjectsTransform(location);

  checker.setCollisionMarginData(CollisionMarginData(0.55));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.55, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  // Bullet: 0.524565 {0.237717,0,0} {0.7622825,0,0} Using blender this appears to be the correct result
  // FCL:    0.546834 {0.237717,-0.0772317,0} {0.7622825,0.0772317}
  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.52448, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.23776)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (0.76224)) > 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.23776, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.76224, 0.001);
  }

  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);
}

inline void runTestConvex3(DiscreteContactManager& checker)
{
  //////////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature face to edge)
  //////////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  tesseract_common::TransformMap location;
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(1) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  // Bullet: -0.280223 {0.0425563,0.2308753,-0.0263040} {-0.0425563, -0.0308753, 0.0263040}
  // FCL:    -0.280223 {0.0425563,0.2308753,-0.0263040} {-0.0425563, -0.0308753, 0.0263040}
  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.280223, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][1] - (0.2308753)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][1] - (-0.0308753)) > 0.001);
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0]), 0.0425563, 0.001);
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2]), 0.0263040, 0.001);
  }
  else
  {
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0]), 0.0425563, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.2308753, 0.001);
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2]), 0.0263040, 0.001);
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0]), 0.0425563, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], -0.0308753, 0.001);
    EXPECT_NEAR(std::abs(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2]), 0.0263040, 0.001);
  }

  EXPECT_NEAR(std::abs(idx[2] * result_vector[0].normal[0]), 0.3037316, 0.001);
  EXPECT_NEAR(idx[2] * result_vector[0].normal[1], 0.9340783, 0.001);
  EXPECT_NEAR(std::abs(idx[2] * result_vector[0].normal[2]), 0.1877358, 0.001);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)))), 0.4);
}

inline void runTestConvex(DiscreteContactManager& checker)
{
  runTestConvex1(checker);
  runTestConvex2(checker);
  runTestConvex3(checker);
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, bool use_convex_mesh)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  if (use_convex_mesh)
    detail::runTestConvex(checker);
  else
    detail::runTestPrimitive(checker);
}
}  // namespace tesseract_collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_SPHERE_SPHERE_UNIT_HPP
