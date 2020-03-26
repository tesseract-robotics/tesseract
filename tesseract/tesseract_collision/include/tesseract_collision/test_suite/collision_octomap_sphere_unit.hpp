#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision
{
namespace test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(TEST_SUITE_DATA_DIR) + "/blender_monkey.bt";
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap = std::make_shared<tesseract_geometry::Octree>(ot, tesseract_geometry::Octree::BOX);
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(TEST_SUITE_DATA_DIR) + "/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

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

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj2_shapes, obj2_poses);
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, double tol, bool use_convex_mesh)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap_link", "sphere_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation() = Eigen::Vector3d(0, 0, 1);
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  auto start_time = std::chrono::high_resolution_clock::now();
  ContactResultMap result;
  for (auto i = 0; i < 10; ++i)
  {
    result.clear();
    checker.contactTest(result, ContactTestType::CLOSEST);
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  CONSOLE_BRIDGE_logInform("DT: %f ms", std::chrono::duration<double, std::milli>(end_time - start_time).count());

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.25, tol);
}
}  // namespace test_suite
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP
