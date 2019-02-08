#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

void runTest(tesseract::DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add Meshed Sphere to checker
  tesseract::CollisionShapePtr sphere;

  if (use_convex_mesh)
  {
    tesseract::VectorVector3d mesh_vertices;
    std::vector<int> mesh_faces;
    EXPECT_GT(tesseract::loadSimplePlyFile(std::string(DATA_DIR) + "/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    std::shared_ptr<tesseract::VectorVector3d> ch_verticies(new tesseract::VectorVector3d());
    std::shared_ptr<std::vector<int>> ch_faces(new std::vector<int>());
    int ch_num_faces = tesseract::createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere.reset(new tesseract::ConvexMeshCollisionShape(ch_verticies, ch_faces, ch_num_faces));
  }
  else
  {
    sphere.reset(new tesseract::SphereCollisionShape(0.25));
  }

  double delta = 0.55;

  std::size_t t = 5;  // Because of unit test runtime this was reduced from 10 to 5.
  std::vector<std::string> link_names;
  tesseract::TransformMap location;
  for (std::size_t x = 0; x < t; ++x)
  {
    for (std::size_t y = 0; y < t; ++y)
    {
      for (std::size_t z = 0; z < t; ++z)
      {
        tesseract::CollisionShapesConst obj3_shapes;
        tesseract::VectorIsometry3d obj3_poses;
        Eigen::Isometry3d sphere_pose;
        sphere_pose.setIdentity();

        obj3_shapes.push_back(tesseract::CollisionShapePtr(sphere->clone()));
        obj3_poses.push_back(sphere_pose);

        link_names.push_back("sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z));

        location[link_names.back()] = sphere_pose;
        location[link_names.back()].translation() = Eigen::Vector3d(
            static_cast<double>(x) * delta, static_cast<double>(y) * delta, static_cast<double>(z) * delta);
        checker.addCollisionObject(link_names.back(), 0, obj3_shapes, obj3_poses);
      }
    }
  }

  // Check if they are in collision
  checker.setActiveCollisionObjects(link_names);
  checker.setContactDistanceThreshold(0.1);
  checker.setCollisionObjectsTransform(location);

  tesseract::ContactResultVector result_vector;

  ros::WallTime start_time = ros::WallTime::now();
  for (auto i = 0; i < 10; ++i)
  {
    tesseract::ContactResultMap result;
    result_vector.clear();
    checker.contactTest(result, tesseract::ContactTestType::ALL);
    tesseract::flattenResults(std::move(result), result_vector);
  }
  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO_STREAM("DT: " << (end_time - start_time).toSec());

  EXPECT_TRUE(result_vector.size() == 300);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetConvexHullUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteSimpleCollisionLargeDataSetUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteSimpleManager checker;
  runTest(checker);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetConvexHullUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, BulletDiscreteBVHCollisionLargeDataSetUnit)
{
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;
  runTest(checker);
}

// TODO: Levi, enable once FCL PR #338
// TEST(TesseractCollisionLargeDataSetUnit, FCLDiscreteBVHCollisionLargeDataSetConvexHullUnit)
//{
//  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
//  runTest(checker, true);
//}

TEST(TesseractCollisionLargeDataSetUnit, FCLDiscreteBVHCollisionLargeDataSetUnit)
{
  tesseract::tesseract_fcl::FCLDiscreteBVHManager checker;
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
