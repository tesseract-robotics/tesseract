
#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

void addCollisionObjects(tesseract::DiscreteContactManagerBase& checker, bool use_convex_mesh = false)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  shapes::ShapePtr sphere;
  if (use_convex_mesh)
    sphere.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  if (use_convex_mesh)
    obj1_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr sphere1;
  if (use_convex_mesh)
    sphere1.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere1.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  tesseract::VectorIsometry3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  if (use_convex_mesh)
    obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses, obj3_types);
}

void runTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  tesseract::ContactRequest req;
  req.link_names.push_back("sphere_link");
  req.link_names.push_back("sphere1_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Test when object is inside another
  tesseract::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

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
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.52;  // 0.251;
  checker.setContactRequest(req);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

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

void runConvexTest(tesseract::DiscreteContactManagerBase& checker)
{
  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  tesseract::ContactRequest req;
  req.link_names.push_back("sphere_link");
  req.link_names.push_back("sphere1_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Test when object is inside another
  tesseract::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.27552, 0.005);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.23776, 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.03776, 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.005);

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ///////////////////////////////////////////////
  // Test object is out side the contact distance
  ///////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Edge to Edge)
  /////////////////////////////////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.55;
  checker.setContactRequest(req);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.52448, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.23776, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.76224, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  //////////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Vertex to Vertex)
  //////////////////////////////////////////////////////////////////////
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(1) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  result.clear();
  result_vector.clear();

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.25, 0.005);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.25, 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], -0.05, 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.005);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.005);

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(0, 1, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Vertex to Vertex)
  /////////////////////////////////////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.51;
  checker.setContactRequest(req);

  // The closest feature of the mesh should be edge to edge
  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionSphereSphereConvexHullUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionSphereSphereConvexHullUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereUnit)
{
  tesseract::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionSphereSphereConvexHullUnit)
{
  tesseract::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runConvexTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
