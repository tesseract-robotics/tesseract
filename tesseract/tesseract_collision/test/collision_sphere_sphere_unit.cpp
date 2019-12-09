#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
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
    std::shared_ptr<tesseract_common::VectorVector3d> ch_verticies(new tesseract_common::VectorVector3d());
    std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere.reset(new ConvexMesh(ch_verticies, ch_faces, ch_num_faces));
  }
  else
  {
    sphere.reset(new Sphere(0.25));
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
  CollisionShapePtr sphere1;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(DATA_DIR) + "/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    std::shared_ptr<tesseract_common::VectorVector3d> ch_verticies(new tesseract_common::VectorVector3d());
    std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere1.reset(new ConvexMesh(ch_verticies, ch_faces, ch_num_faces));
  }
  else
  {
    sphere1.reset(new Sphere(0.25));
  }

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
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0.1);

  // Test when object is inside another
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.30, 0.0001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
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
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.52);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.0001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

void runConvexTest(DiscreteContactManager& checker)
{
  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setContactDistanceThreshold(0.1);

  // Test when object is inside another
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  // Bullet: -0.270548 {0.232874,0,-0.025368} {-0.032874,0,0.025368}
  // FCL:    -0.270548 {0.232874,0,-0.025368} {-0.032874,0,0.025368}
  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.270548, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.232874, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], -0.025368, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.032874, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.025368, 0.001);
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

  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());
}

void runConvexTest2(DiscreteContactManager& checker)
{
  /////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Edge to Edge)
  /////////////////////////////////////////////////////////////////////////
  ContactResultMap result;
  ContactResultVector result_vector;

  checker.setContactDistanceThreshold(0.55);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  // Bullet: 0.524565 {0.237717,0,0} {0.7622825,0,0} Using blender this appears to be the correct result
  // FCL:    0.546834 {0.237717,-0.0772317,0} {0.7622825,0.0772317}
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
}

void runConvexTest3(DiscreteContactManager& checker)
{
  //////////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature face to edge)
  //////////////////////////////////////////////////////////////////////
  tesseract_common::TransformMap location;
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(1) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  ContactResultVector result_vector;
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  // Bullet: -0.280223 {0.0425563,0.2308753,-0.0263040} {-0.0425563, -0.0308753, 0.0263040}
  // FCL:    -0.280223 {0.0425563,0.2308753,-0.0263040} {-0.0425563, -0.0308753, 0.0263040}
  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.280223, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(std::abs(result_vector[0].nearest_points[idx[0]][0]), 0.0425563, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.2308753, 0.001);
  EXPECT_NEAR(std::abs(result_vector[0].nearest_points[idx[0]][2]), 0.0263040, 0.001);
  EXPECT_NEAR(std::abs(result_vector[0].nearest_points[idx[1]][0]), 0.0425563, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], -0.0308753, 0.001);
  EXPECT_NEAR(std::abs(result_vector[0].nearest_points[idx[1]][2]), 0.0263040, 0.001);
  EXPECT_NEAR(std::abs(idx[2] * result_vector[0].normal[0]), 0.3037316, 0.001);
  EXPECT_NEAR(idx[2] * result_vector[0].normal[1], 0.9340783, 0.001);
  EXPECT_NEAR(std::abs(idx[2] * result_vector[0].normal[2]), 0.1877358, 0.001);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)))), 0.4);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
  runConvexTest2(checker);
  runConvexTest3(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
  runConvexTest2(checker);
  runConvexTest3(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
  //  runConvexTest2(checker); FCL return incorrect results, need to create an issue on FCL
  runConvexTest3(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
