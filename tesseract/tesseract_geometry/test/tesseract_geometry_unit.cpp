#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>

TEST(TesseractGeometryUnit, Instantiation)  // NOLINT
{
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices =
      std::make_shared<const tesseract_common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> faces = std::make_shared<const Eigen::VectorXi>();

  auto box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);
  auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);
  auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);
  auto sphere = std::make_shared<tesseract_geometry::Sphere>(1);
  auto convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(vertices, faces);
  auto mesh = std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  auto sdf_mesh = std::make_shared<tesseract_geometry::SDFMesh>(vertices, faces);
  auto octree = std::make_shared<tesseract_geometry::Octree>(nullptr, tesseract_geometry::Octree::SubType::BOX);

  // Instead making this depend on pcl it expects the structure to have a member called points which is a vector
  // of another object with has float members x, y and z.
  struct TestPointCloud
  {
    struct point
    {
      double x;
      double y;
      double z;
    };

    std::vector<point> points;
  };

  TestPointCloud pc;
  auto octree_pc =
      std::make_shared<tesseract_geometry::Octree>(pc, 0.01, tesseract_geometry::Octree::SubType::BOX, false);
}

TEST(TesseractGeometryUnit, LoadMeshUnit)  // NOLINT
{
  using namespace tesseract_geometry;

  std::string mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.stl";
  std::vector<Mesh::Ptr> meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 80);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 42);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply";
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 80);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 42);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.dae";
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 2);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 80);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 42);
  EXPECT_TRUE(meshes[1]->getTriangleCount() == 80);
  EXPECT_TRUE(meshes[1]->getVerticeCount() == 42);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.dae";
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 2 * 80);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 2 * 42);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box_2m.ply";
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 12);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 8);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box_2m.ply";
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getTriangleCount() == 12);
  EXPECT_TRUE(meshes[0]->getVerticeCount() == 8);

  mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box_2m.ply";
  std::vector<ConvexMesh::Ptr> convex_meshes =
      createMeshFromPath<ConvexMesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, false);
  EXPECT_TRUE(convex_meshes.size() == 1);
  EXPECT_TRUE(convex_meshes[0]->getFaceCount() == 6);
  EXPECT_TRUE(convex_meshes[0]->getVerticeCount() == 8);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
