
#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

void runTest(tesseract::DiscreteContactManagerBase &checker, bool use_convex_mesh = false)
{
  // Add Meshed Sphere to checker
  shapes::ShapePtr sphere;
  if (use_convex_mesh)
    sphere.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere.reset(new shapes::Sphere(0.25));

  double delta = 0.55;

  std::size_t t = 10;
  std::vector<std::string> link_names;
  tesseract::TransformMap location;
  for (std::size_t x = 0; x < t; ++x)
  {
    for (std::size_t y = 0; y < t; ++y)
    {
      for (std::size_t z = 0; z < t; ++z)
      {
        std::vector<shapes::ShapeConstPtr> obj3_shapes;
        EigenSTL::vector_Affine3d obj3_poses;
        tesseract::CollisionObjectTypeVector obj3_types;
        Eigen::Affine3d sphere_pose;
        sphere_pose.setIdentity();

        obj3_shapes.push_back(shapes::ShapePtr(sphere->clone()));
        obj3_poses.push_back(sphere_pose);

        if (use_convex_mesh)
          obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
        else
          obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

        link_names.push_back("sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z));

        location[link_names.back()] = sphere_pose;
        location[link_names.back()].translation() = Eigen::Vector3d(x * delta, y * delta, z * delta);
        checker.addCollisionObject(link_names.back(), 0, obj3_shapes, obj3_poses, obj3_types);
      }
    }
  }

  // Check if they are in collision
  tesseract::ContactRequest req;
  req.link_names = link_names;
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::ALL;
  checker.setContactRequest(req);
  checker.setCollisionObjectsTransform(location);

  tesseract::ContactResultVector result_vector;

  ros::WallTime start_time = ros::WallTime::now();
  for (auto i = 0; i < 10; ++i)
  {
    tesseract::ContactResultMap result;
    result_vector.clear();
    checker.contactTest(result);
    tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
  }
  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO_STREAM("DT: " << (end_time - start_time).toSec());

  EXPECT_TRUE(result_vector.size() == 2700);

//  checker.calcCollisionsContinuous(req, location, location2, result);
//  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetConvexHullUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  runTest(checker);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetConvexHullUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
