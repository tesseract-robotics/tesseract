#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <gtest/gtest.h>
#include <chrono>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(DATA_DIR) + "/blender_monkey.bt";
  std::shared_ptr<octomap::OcTree> ot(new octomap::OcTree(path));
  CollisionShapePtr dense_octomap(new Octree(ot, Octree::SubType::BOX));
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

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj2_shapes, obj2_poses);
}

void runTest(DiscreteContactManager& checker, double tol)
{
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

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker, true);
  runTest(checker, 0.02);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runTest(checker, 0.02);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.16);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  addCollisionObjects(checker, true);
  runTest(checker, 0.16);  // TODO: There appears to be an issue in fcl for octomap::OcTree.
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
