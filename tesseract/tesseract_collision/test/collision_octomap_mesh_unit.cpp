#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <gtest/gtest.h>
#include <tesseract_geometry/mesh_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"
#include "tesseract_collision/core/common.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(DiscreteContactManager& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(DATA_DIR) + "/box_2m.bt";
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
  // Add plane mesh to checker.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere = createMeshFromPath<tesseract_geometry::Mesh>(
      std::string(DATA_DIR) + "/plane_4m.stl", Eigen::Vector3d(1, 1, 1), true)[0];

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("plane_link", 0, obj2_shapes, obj2_poses);
}

void runTest(DiscreteContactManager& checker, double /*tol*/, const std::string& file_path)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap_link", "plane_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["plane_link"] = Eigen::Isometry3d::Identity();
  location["plane_link"].translation() = Eigen::Vector3d(0, 0, 0);
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::ALL);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  const tesseract_collision::CollisionShapesConst& geom = checker.getCollisionObjectGeometries("plane_link");
  const auto& mesh = std::static_pointer_cast<const tesseract_geometry::Mesh>(geom.at(0));
  const auto& mesh_vertices = mesh->getVertices();
  const auto& mesh_triangles = mesh->getTriangles();

  // default color is green
  std::vector<Eigen::Vector3i> mesh_vertices_color(mesh_vertices->size(), Eigen::Vector3i(0, 128, 0));

  for (auto& r : result_vector)
  {
    int idx = 0;
    if (r.link_names[0] != "plane_link")
      idx = 1;

    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 1])] =
        Eigen::Vector3i(255, 0, 0);
    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 2])] =
        Eigen::Vector3i(255, 0, 0);
    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 3])] =
        Eigen::Vector3i(255, 0, 0);
  }

  writeSimplePlyFile(file_path, *mesh_vertices, mesh_vertices_color, *mesh_triangles, mesh->getTriangleCount());

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_TRUE(result_vector.size() == 2712);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001, "/tmp/BulletDiscreteSimpleCollisionOctomapSphereMeshUnit.ply");
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereMeshUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, 0.001, "/tmp/BulletDiscreteBVHCollisionOctomapSphereMeshUnit.ply");
}

// TODO: There is an issue with fcl octomap collision type. Either with tesseract implementation or fcl
// TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionOctomapSphereMeshUnit)
//{
//  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
//  addCollisionObjects(checker);
//  runTest(checker, 0.16, "/tmp/FCLDiscreteBVHCollisionOctomapSphereMeshUnit.ply");  // TODO: There appears to be an
//  issue in fcl for octomap::OcTree.
//}

/** @brief This is to test the shape id and subshape id of the contact results. */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
