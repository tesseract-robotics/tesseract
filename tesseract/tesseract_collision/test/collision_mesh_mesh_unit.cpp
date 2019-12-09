#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(DiscreteContactManager& checker)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  Mesh::Ptr sphere;

  std::shared_ptr<tesseract_common::VectorVector3d> vertices(new tesseract_common::VectorVector3d());
  std::shared_ptr<Eigen::VectorXi> faces(new Eigen::VectorXi());
  int num_faces = loadSimplePlyFile(std::string(DATA_DIR) + "/sphere_p25m.ply", *vertices, *faces, true);
  EXPECT_GT(num_faces, 0);

  sphere.reset(new Mesh(vertices, faces));
  EXPECT_TRUE(num_faces == sphere->getTriangleCount());

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
  CollisionShapePtr thin_box(new Box(0.1, 1, 1));
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
  CollisionShapePtr sphere1(new Mesh(vertices, faces));
  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);
}

void runTest(DiscreteContactManager& checker)
{
  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0);

  // Test when object is inside another
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::ALL);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.size() >= 37);

  ///////////////////////////////////////////////
  // Test object is out side the contact distance
  ///////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactTestType::ALL);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Edge to Edge)
  /////////////////////////////////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.55);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.52448, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.23776, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.76224, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);

  /////////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Vertex to Vertex)
  /////////////////////////////////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(0, 1, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  // The closest feature of the mesh should be edge to edge
  checker.setCollisionObjectsTransform(location);
  checker.setContactDistanceThreshold(0.55);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)))), 0.00001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionMeshMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionMeshMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionMeshMeshUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
