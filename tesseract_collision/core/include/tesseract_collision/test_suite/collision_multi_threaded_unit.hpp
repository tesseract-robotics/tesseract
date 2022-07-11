#ifndef TESSERACT_COLLISION_COLLISION_MULTI_THREADED_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_MULTI_THREADED_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision::test_suite
{
inline void runTest(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add Meshed Sphere to checker
  CollisionShapePtr sphere;
  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(
        loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces),
        0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
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
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);
  checker.setCollisionObjectsTransform(location);

  long num_threads = 4;
  std::vector<ContactResultVector> result_vector(static_cast<std::size_t>(num_threads));
  std::vector<DiscreteContactManager::Ptr> contact_manager(static_cast<std::size_t>(num_threads));
  contact_manager[0] = checker.clone();
  contact_manager[1] = checker.clone();
  contact_manager[2] = checker.clone();
  contact_manager[3] = checker.clone();

  auto start_time = std::chrono::high_resolution_clock::now();

#pragma omp parallel for num_threads(num_threads) shared(location)
  for (long i = 0; i < num_threads; ++i)  // NOLINT
  {
    const int tn = omp_get_thread_num();
    CONSOLE_BRIDGE_logDebug("Thread (ID: %i): %i of %i", tn, i, num_threads);
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
        std::vector<std::string> names = { co.first };
        tesseract_common::VectorIsometry3d transforms = { pose };
        manager->setCollisionObjectsTransform(names, transforms);
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
        std::vector<std::string> names = { co.first };
        tesseract_common::VectorIsometry3d transforms = { pose };
        manager->setCollisionObjectsTransform(names, transforms);
      }
    }

    ContactResultMap result;
    manager->contactTest(result, ContactRequest(ContactTestType::ALL));
    flattenResults(std::move(result), result_vector[static_cast<size_t>(tn)]);
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  CONSOLE_BRIDGE_logInform("DT: %f ms", std::chrono::duration<double, std::milli>(end_time - start_time).count());

  for (long i = 0; i < num_threads; ++i)
  {
    EXPECT_TRUE(result_vector[static_cast<std::size_t>(i)].size() == 2700);
  }
}
}  // namespace tesseract_collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_MULTI_THREADED_UNIT_HPP
