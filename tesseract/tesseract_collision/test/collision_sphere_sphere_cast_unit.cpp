#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(ContinuousContactManager& checker, bool use_convex_mesh = false)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  CollisionShapePtr sphere;
  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(DATA_DIR) + "/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_vertices, *ch_faces, mesh_vertices);
    sphere = std::make_shared<ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<Sphere>(0.25);
  }

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation()[2] = 0.25;

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(DATA_DIR) + "/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere1 = std::make_shared<ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere1 = std::make_shared<Sphere>(0.25);
  }

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();
  sphere1_pose.translation()[2] = 0.25;

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);
}

void runTest(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  ///////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0.1);

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
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.1, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[idx[0]], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[idx[1]], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][0], -0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[idx[0]].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[idx[1]].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[0]].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[1]].isApprox(location_end["sphere1_link"], 0.0001));

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
  checker.contactTest(result, ContactTestType::CLOSEST);

  result_vector = ContactResultVector();
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.1, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[idx[0]], 0.3333, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[idx[1]], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[1])] == ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][0], -0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[idx[0]].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[idx[1]].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[0]].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[1]].isApprox(location_end["sphere1_link"], 0.0001));

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

void runConvexTest(ContinuousContactManager& checker)
{
  ///////////////////////////////////////////////////
  // Test when object is in collision at cc_time 0.5
  ///////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0.1);

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
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.0754, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[idx[0]], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[idx[1]], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][0], 0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][0], -0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[idx[0]].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[idx[1]].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[0]].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[1]].isApprox(location_end["sphere1_link"], 0.0001));

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
  checker.contactTest(result, ContactTestType::CLOSEST);

  result_vector = ContactResultVector();
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.0754, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].cc_time[idx[0]], 0.3848, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time[idx[1]], 0.5, 0.001);

  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[0])] == ContinuousCollisionType::CCType_Between);
  EXPECT_TRUE(result_vector[0].cc_type[static_cast<size_t>(idx[1])] == ContinuousCollisionType::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0772, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.0377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0772, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.25, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][0], 0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[0]][2], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][0], -0.2377, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points_local[idx[1]][2], 0.25, 0.001);

  EXPECT_TRUE(result_vector[0].transform[idx[0]].isApprox(location_start["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].transform[idx[1]].isApprox(location_start["sphere1_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[0]].isApprox(location_end["sphere_link"], 0.0001));
  EXPECT_TRUE(result_vector[0].cc_transform[idx[1]].isApprox(location_end["sphere1_link"], 0.0001));

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
