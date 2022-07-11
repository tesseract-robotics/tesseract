#ifndef TESSERACT_COLLISION_COLLISION_BOX_BOX_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_BOX_BOX_UNIT_HPP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  CollisionShapePtr box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("box_link");

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

  /////////////////////////////////////////////////////////////////
  // Add second box to checker. If use_convex_mesh = true then this
  // box will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr second_box;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    // TODO: Need to figure out why this test not pass of bullet when using the box_2m.ply mesh
    EXPECT_GT(loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box2_2m.ply", mesh_vertices, mesh_faces),
              0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    second_box = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    second_box = std::make_shared<tesseract_geometry::Box>(2, 2, 2);
  }

  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);

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

inline void runTestTyped(DiscreteContactManager& checker, ContactTestType test_type)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getPairCollisionMargin("box_link", "second_box_link"), 0.1, 1e-5);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -1.30, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (-0.3)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (1.0)) > 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], -0.3, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 1.0, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is outside the contact distance
  ////////////////////////////////////////////////
  {
    location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
    result = ContactResultMap();
    result.clear();
    result_vector.clear();

    // Use different method for setting transforms
    std::vector<std::string> names = { "box_link" };
    tesseract_common::VectorIsometry3d transforms = { location["box_link"] };
    checker.setCollisionObjectsTransform(names, transforms);
    checker.contactTest(result, test_type);
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(result_vector.empty());
  }
  ////////////////////////////////////////////////
  // Test object is outside the contact distance only for this link pair
  ////////////////////////////////////////////////
  {
    CollisionMarginData data = checker.getCollisionMarginData();
    data.setPairCollisionMargin("not_box_link", "also_not_box_link", 1.7);
    checker.setCollisionMarginData(data);

    EXPECT_EQ(checker.getCollisionMarginData().getMaxCollisionMargin(), 1.7);
    EXPECT_NEAR(checker.getCollisionMarginData().getPairCollisionMargin("box_link", "second_box_link"), 0.1, 1e-5);
    location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
    result = ContactResultMap();
    result.clear();
    result_vector.clear();

    // Use different method for setting transforms
    std::vector<std::string> names = { "box_link" };
    tesseract_common::VectorIsometry3d transforms = { location["box_link"] };
    checker.setCollisionObjectsTransform(names, transforms);
    checker.contactTest(result, test_type);
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(result_vector.empty());
  }
  /////////////////////////////////////////////
  // Test object inside the contact distance only for this link pair
  /////////////////////////////////////////////
  {
    result = ContactResultMap();
    result.clear();
    result_vector.clear();

    CollisionMarginData data(0.1);
    data.setPairCollisionMargin("box_link", "second_box_link", 0.25);

    checker.setCollisionMarginData(data);
    EXPECT_NEAR(checker.getCollisionMarginData().getPairCollisionMargin("box_link", "second_box_link"), 0.25, 1e-5);
    EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);
    checker.contactTest(result, ContactRequest(test_type));
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(!result_vector.empty());
    EXPECT_NEAR(result_vector[0].distance, 0.1, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

    idx = { 0, 1, 1 };
    if (result_vector[0].link_names[0] != "box_link")
      idx = { 1, 0, -1 };

    if (result_vector[0].single_contact_point)
    {
      EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
      EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (1.1)) > 0.001 &&
                   std::abs(result_vector[0].nearest_points[0][0] - (1.0)) > 0.001);
    }
    else
    {
      EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 1.1, 0.001);
      EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 1.0, 0.001);
    }

    EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
    EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
  }
  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  {
    result = ContactResultMap();
    result.clear();
    result_vector.clear();

    checker.setCollisionMarginData(CollisionMarginData(0.25));
    EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.25, 1e-5);
    checker.contactTest(result, ContactRequest(test_type));
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(!result_vector.empty());
    EXPECT_NEAR(result_vector[0].distance, 0.1, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

    idx = { 0, 1, 1 };
    if (result_vector[0].link_names[0] != "box_link")
      idx = { 1, 0, -1 };

    if (result_vector[0].single_contact_point)
    {
      EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
      EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (1.1)) > 0.001 &&
                   std::abs(result_vector[0].nearest_points[0][0] - (1.0)) > 0.001);
    }
    else
    {
      EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 1.1, 0.001);
      EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 1.0, 0.001);
    }

    EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
    EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
  }
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  detail::runTestTyped(checker, ContactTestType::FIRST);
  detail::runTestTyped(checker, ContactTestType::CLOSEST);
  detail::runTestTyped(checker, ContactTestType::ALL);
}

}  // namespace tesseract_collision::test_suite
#endif  // COLLISION_BOX_BOX_UNIT_HPP
