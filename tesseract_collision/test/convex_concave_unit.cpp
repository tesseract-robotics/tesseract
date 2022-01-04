#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_contact_checker.h"

TEST(TesseractConvexConcaveUnit, ConvexConcaveUnit)
{
  tesseract::BulletContactChecker checker;

  // Add box to checker
  tesseract::CollisionShapePtr box = std::make_shared<shapes::Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  tesseract::CollisionShapesConst obj1_shapes;
  VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  // Add box to checker
  tesseract::CollisionShapePtr thin_box = std::make_shared<shapes::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  tesseract::CollisionShapesConst obj2_shapes;
  VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types);

  // Add Meshed Sphere to checker
  tesseract::CollisionShapePtr sphere(shapes::createMeshFromResource("package://tesseract_collision/test/sphere.stl"));
  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  tesseract::CollisionShapesConst obj3_shapes;
  VectorIsometry3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);
  // Since this is a mesh this will be considered a concave shape
  obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("sphere_link", 0, obj3_shapes, obj3_poses, obj3_types);

  // Check if they are in collision
  tesseract::ContactRequest req;
  req.link_names.push_back("box_link");
  req.link_names.push_back("sphere_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;

  // Test when object is inside another
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();

  tesseract::ContactResultMap result;
  checker.calcCollisionsDiscrete(req, location, result);

  tesseract::ContactResultVector result_vector;
  tesseract::flattenResults(result, result_vector);

  // This does fail need to create an issue on bullet
  EXPECT_LT(std::abs(result_vector[0].distance + 0.75), 0.0001);
  EXPECT_TRUE(!result_vector.empty());

  // Test object is out side the contact distance
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();

  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::flattenResults(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  // Test object right at the contact distance
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.251;

  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::flattenResults(result, result_vector);

  EXPECT_LT(std::abs(0.25 - result_vector[0].distance), 0.0001);
  EXPECT_TRUE(!result_vector.empty());

  //  // Test Cast object
  //  result.clear();
  //  result_vector.clear();
  //  tesseract::TransformMap location2;
  //  location.clear();
  //  location["thin_box_link"] = Eigen::Isometry3d::Identity();
  //  location["sphere_link"] = Eigen::Isometry3d::Identity();
  //  location2["thin_box_link"] = Eigen::Isometry3d::Identity();
  //  location2["sphere_link"] = Eigen::Isometry3d::Identity();

  //  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  //  location2["sphere_link"].translation() = Eigen::Vector3d(-1, 0, 0);

  //  req.link_names.clear();
  //  req.link_names.push_back("sphere_link");
  //  req.contact_distance = 0.1;
  //  req.type = tesseract::ContactRequestType::SINGLE;

  //  checker.calcCollisionsContinuous(req, location, location2, result);
  //  tesseract::flattenResults(result,
  //  result_vector);
  //  EXPECT_TRUE(!result_vector.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
