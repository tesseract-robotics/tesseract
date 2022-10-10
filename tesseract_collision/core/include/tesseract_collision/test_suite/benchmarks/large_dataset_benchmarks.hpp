#ifndef TESSERACT_COLLISION_LARGE_DATASET_BENCHMARKS_HPP
#define TESSERACT_COLLISION_LARGE_DATASET_BENCHMARKS_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision
{
namespace test_suite
{
/** @brief Benchmark that checks collisions between a lot of objects. In this case it is a grid of spheres - each as its
 * own link*/
static void BM_LARGE_DATASET_MULTILINK(benchmark::State& state,
                                       DiscreteContactManager::Ptr checker,  // NOLINT
                                       int edge_size,
                                       tesseract_geometry::GeometryType type)
{
  // Add Meshed Sphere to checker
  CollisionShapePtr sphere;

  tesseract_common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces);

  // This is required because convex hull cannot have multiple faces on the same plane.
  auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
  auto ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);

  switch (type)
  {
    case tesseract_geometry::GeometryType::CONVEX_MESH:
      sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
      break;
    case tesseract_geometry::GeometryType::MESH:
      sphere = std::make_shared<tesseract_geometry::Mesh>(ch_verticies, ch_faces, ch_num_faces);
      break;
    case tesseract_geometry::GeometryType::SPHERE:
      sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
      break;
    default:
      throw(std::runtime_error("Invalid geometry type"));
      break;
  }

  double delta = 0.55;

  std::vector<std::string> link_names;
  tesseract_common::TransformMap location;
  for (int x = 0; x < edge_size; ++x)
  {
    for (int y = 0; y < edge_size; ++y)
    {
      for (int z = 0; z < edge_size; ++z)
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
        checker->addCollisionObject(link_names.back(), 0, obj3_shapes, obj3_poses);
      }
    }
  }

  // Check if they are in collision
  checker->setActiveCollisionObjects(link_names);
  checker->setCollisionMarginData(CollisionMarginData(0.1));
  checker->setCollisionObjectsTransform(location);

  ContactResultVector result_vector;

  for (auto _ : state)  // NOLINT
  {
    ContactResultMap result;
    result_vector.clear();
    checker->contactTest(result, ContactTestType::ALL);
    flattenMoveResults(std::move(result), result_vector);
  }
};

/** @brief Benchmark that checks collisions between a lot of objects. In this case it is a grid of spheres in one link
 * and a single sphere in another link*/
static void BM_LARGE_DATASET_SINGLELINK(benchmark::State& state,
                                        DiscreteContactManager::Ptr checker,  // NOLINT
                                        int edge_size,
                                        tesseract_geometry::GeometryType type)
{
  // Add Meshed Sphere to checker
  CollisionShapePtr sphere;

  tesseract_common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces);

  // This is required because convex hull cannot have multiple faces on the same plane.
  auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
  auto ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);

  switch (type)
  {
    case tesseract_geometry::GeometryType::CONVEX_MESH:
      sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
      break;
    case tesseract_geometry::GeometryType::MESH:
      sphere = std::make_shared<tesseract_geometry::Mesh>(ch_verticies, ch_faces, ch_num_faces);
      break;
    case tesseract_geometry::GeometryType::SPHERE:
      sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
      break;
    default:
      throw(std::runtime_error("Invalid geometry type"));
      break;
  }

  // Add Grid of spheres
  double delta = 0.55;

  std::vector<std::string> link_names;
  //  tesseract_common::TransformMap location;
  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  for (int x = 0; x < edge_size; ++x)
  {
    for (int y = 0; y < edge_size; ++y)
    {
      for (int z = 0; z < edge_size; ++z)
      {
        Eigen::Isometry3d sphere_pose;
        sphere_pose.setIdentity();
        sphere_pose.translation() = Eigen::Vector3d(
            static_cast<double>(x) * delta, static_cast<double>(y) * delta, static_cast<double>(z) * delta);

        obj3_shapes.push_back(CollisionShapePtr(sphere->clone()));
        obj3_poses.push_back(sphere_pose);
      }
    }
  }
  link_names.emplace_back("grid_link");
  checker->addCollisionObject(link_names.back(), 0, obj3_shapes, obj3_poses);

  // Add Single Sphere Link
  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation() = Eigen::Vector3d(static_cast<double>(edge_size) / 2.0 * delta,
                                              static_cast<double>(edge_size) / 2.0 * delta,
                                              static_cast<double>(edge_size) / 2.0 * delta);
  CollisionShapesConst single_shapes;
  tesseract_common::VectorIsometry3d single_poses;
  single_shapes.push_back(CollisionShapePtr(sphere->clone()));
  single_poses.push_back(sphere_pose);
  link_names.emplace_back("single_link");
  checker->addCollisionObject(link_names.back(), 0, single_shapes, single_poses);

  // Check if they are in collision
  checker->setActiveCollisionObjects(link_names);
  checker->setCollisionMarginData(CollisionMarginData(0.1));
  //  checker->setCollisionObjectsTransform(location);

  ContactResultVector result_vector;

  for (auto _ : state)  // NOLINT
  {
    ContactResultMap result;
    result_vector.clear();
    checker->contactTest(result, ContactTestType::ALL);
    flattenMoveResults(std::move(result), result_vector);
  }
};

}  // namespace test_suite
}  // namespace tesseract_collision

#endif
