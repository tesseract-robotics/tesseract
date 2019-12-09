#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <console_bridge/console.h>
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void runTest(tesseract_collision::DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add Meshed Sphere to checker
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

  double delta = 0.55;

  std::size_t t = 10;
  std::vector<std::string> link_names;
  tesseract_common::TransformMap location;
  for (std::size_t x = 0; x < t; ++x)
  {
    for (std::size_t y = 0; y < t; ++y)
    {
      for (std::size_t z = 0; z < t; ++z)
      {
        CollisionShapesConst obj3_shapes;
        tesseract_common::VectorIsometry3d obj3_poses;
        Eigen::Isometry3d sphere_pose;
        sphere_pose.setIdentity();

        obj3_shapes.push_back(CollisionShapePtr(sphere->clone()));
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

  unsigned num_threads = 4;
  std::vector<ContactResultVector> result_vector(num_threads);
  std::vector<DiscreteContactManager::Ptr> contact_manager(num_threads);
  contact_manager[0] = checker.clone();
  contact_manager[1] = checker.clone();
  contact_manager[2] = checker.clone();
  contact_manager[3] = checker.clone();

  auto start_time = std::chrono::high_resolution_clock::now();

#pragma omp parallel for num_threads(num_threads) shared(location)
  for (unsigned i = 0; i < num_threads; ++i)
  {
    const int tn = omp_get_thread_num();
    CONSOLE_BRIDGE_logDebug("Thread %i of %i", tn, omp_get_num_threads());
    const DiscreteContactManager::Ptr& manager = contact_manager[static_cast<size_t>(tn)];
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

    ContactResultMap result;
    manager->contactTest(result, ContactTestType::ALL);
    flattenResults(std::move(result), result_vector[static_cast<size_t>(tn)]);
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  CONSOLE_BRIDGE_logInform("DT: %f ms", std::chrono::duration<double, std::milli>(end_time - start_time).count());

  for (unsigned i = 0; i < num_threads; ++i)
  {
    EXPECT_TRUE(result_vector[i].size() == 2700);
  }
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteSimpleCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, BulletDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  runTest(checker);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  runTest(checker, true);
}

TEST(TesseractCollisionMultiThreadedUnit, FCLDiscreteBVHCollisionMultiThreadedUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
