#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  tesseract_common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract_support/meshes/blender_monkey.bt")->getFilePath();
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap =
      std::make_shared<tesseract_geometry::Octree>(ot, tesseract_geometry::OctreeSubType::BOX);
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
    auto mesh_vertices = std::make_shared<tesseract_common::VectorVector3d>();
    auto mesh_faces = std::make_shared<Eigen::VectorXi>();
    EXPECT_GT(
        loadSimplePlyFile(locator.locateResource("package://tesseract_support/meshes/sphere_p25m.ply")->getFilePath(),
                          *mesh_vertices,
                          *mesh_faces,
                          true),
        0);

    auto mesh = std::make_shared<tesseract_geometry::Mesh>(mesh_vertices, mesh_faces);
    sphere = makeConvexMesh(*mesh);
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

  EXPECT_TRUE(checker.getCollisionObjects().size() == 2);
  for (const auto& co : checker.getCollisionObjects())
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co).size() == 1);
    for (const auto& cgt : checker.getCollisionObjectGeometriesTransforms(co))
    {
      EXPECT_TRUE(cgt.isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
  }
}

inline void runTestTyped(DiscreteContactManager& checker, double tol, ContactTestType test_type)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  std::vector<std::string> active_links{ "octomap_link", "sphere_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract_common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getIsContactAllowedFn() == nullptr);

  checker.setDefaultCollisionMarginData(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation() = Eigen::Vector3d(0, 0, 1);
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(test_type));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  if (test_type == ContactTestType::CLOSEST)
  {
    EXPECT_TRUE(result_vector.size() == 1);
    EXPECT_NEAR(result_vector[0].distance, -0.25, tol);
  }
  else if (test_type == ContactTestType::FIRST)
  {
    EXPECT_TRUE(result_vector.size() == 1);
    EXPECT_TRUE(result_vector[0].distance < 0.1);
  }
  else
  {
    EXPECT_FALSE(result_vector.empty());
    EXPECT_TRUE(result_vector[0].distance < 0.1);
  }
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, double tol, bool use_convex_mesh)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  // Call it again to test adding same object
  detail::addCollisionObjects(checker, use_convex_mesh);

  detail::runTestTyped(checker, tol, ContactTestType::FIRST);
  detail::runTestTyped(checker, tol, ContactTestType::CLOSEST);
  detail::runTestTyped(checker, tol, ContactTestType::ALL);
}

}  // namespace tesseract_collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_SPHERE_UNIT_HPP
