
#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

void runTest(tesseract::DiscreteContactManagerBase& checker, bool use_convex_mesh = false)
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
        tesseract::VectorIsometry3d obj3_poses;
        tesseract::CollisionObjectTypeVector obj3_types;
        Eigen::Isometry3d sphere_pose;
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

  unsigned num_threads = 4;
  std::vector<tesseract::ContactResultVector> result_vector(num_threads);
  std::vector<tesseract::DiscreteContactManagerBasePtr> contact_manager(num_threads);
  contact_manager[0] = checker.clone();
  contact_manager[1] = checker.clone();
  contact_manager[2] = checker.clone();
  contact_manager[3] = checker.clone();

  ros::WallTime start_time = ros::WallTime::now();

#pragma omp parallel for num_threads(num_threads) shared(location)
  for (unsigned i = 0; i < num_threads; ++i)
  {
    const int tn = omp_get_thread_num();
    ROS_DEBUG("Thread %i of %i", tn, omp_get_num_threads());
    const tesseract::DiscreteContactManagerBasePtr& manager = contact_manager[tn];
    for (const auto& co : location)
    {
      if (tn == 0)
      {
        Eigen::Isometry3d pose = co.second;
        pose.translation()[0] += 0.1;
        manager->setCollisionObjectsTransform(co.first, pose);
      }
      else if (tn == 1)
      {
        Eigen::Isometry3d pose = co.second;
        pose.translation()[1] += 0.1;
        manager->setCollisionObjectsTransform(co.first, pose);
      }
      else if (tn == 2)
      {
        Eigen::Isometry3d pose = co.second;
        pose.translation()[2] += 0.1;
        manager->setCollisionObjectsTransform(co.first, pose);
      }
      else
      {
        Eigen::Isometry3d pose = co.second;
        pose.translation()[0] -= 0.1;
        manager->setCollisionObjectsTransform(co.first, pose);
      }
    }

    tesseract::ContactResultMap result;
    manager->contactTest(result);
    tesseract::moveContactResultsMapToContactResultsVector(result, result_vector[tn]);
  }
  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO_STREAM("DT: " << (end_time - start_time).toSec());

  for (unsigned i = 0; i < num_threads; ++i)
  {
    EXPECT_TRUE(result_vector[i].size() == 2700);
  }
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedConvexHullUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedConvexHullUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedUnit)
{
  tesseract::FCLDiscreteBVHManager checker;
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
