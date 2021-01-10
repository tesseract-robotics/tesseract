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
  auto capsule = std::make_shared<tesseract_geometry::Capsule>(1, 1);
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

TEST(TesseractGeometryUnit, Box)  // NOLINT
{
  using T = tesseract_geometry::Box;
  auto geom = std::make_shared<T>(1, 1, 1);
  EXPECT_NEAR(geom->getX(), 1, 1e-5);
  EXPECT_NEAR(geom->getY(), 1, 1e-5);
  EXPECT_NEAR(geom->getZ(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getX(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getY(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getZ(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, Cone)  // NOLINT
{
  using T = tesseract_geometry::Cone;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, Cylinder)  // NOLINT
{
  using T = tesseract_geometry::Cylinder;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, Capsule)  // NOLINT
{
  using T = tesseract_geometry::Capsule;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, Sphere)  // NOLINT
{
  using T = tesseract_geometry::Sphere;
  auto geom = std::make_shared<T>(1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, Plane)  // NOLINT
{
  using T = tesseract_geometry::Plane;
  auto geom = std::make_shared<T>(1, 1, 1, 1);
  EXPECT_NEAR(geom->getA(), 1, 1e-5);
  EXPECT_NEAR(geom->getB(), 1, 1e-5);
  EXPECT_NEAR(geom->getC(), 1, 1e-5);
  EXPECT_NEAR(geom->getD(), 1, 1e-5);

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getA(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getB(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getC(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getD(), 1, 1e-5);
}

TEST(TesseractGeometryUnit, ConvexMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(5);
  (*faces)(0) = 4;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;
  (*faces)(4) = 3;

  using T = tesseract_geometry::ConvexMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVerticeCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 1);

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVerticeCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 1);
}

TEST(TesseractGeometryUnit, Mesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces)(0) = 3;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;

  (*faces)(4) = 3;
  (*faces)(5) = 0;
  (*faces)(6) = 2;
  (*faces)(7) = 3;

  using T = tesseract_geometry::Mesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getTriangles() != nullptr);
  EXPECT_TRUE(geom->getVerticeCount() == 4);
  EXPECT_TRUE(geom->getTriangleCount() == 2);

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getTriangles() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVerticeCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getTriangleCount() == 2);
}

TEST(TesseractGeometryUnit, SDFMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces)(0) = 3;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;

  (*faces)(4) = 3;
  (*faces)(5) = 0;
  (*faces)(6) = 2;
  (*faces)(7) = 3;

  using T = tesseract_geometry::SDFMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getTriangles() != nullptr);
  EXPECT_TRUE(geom->getVerticeCount() == 4);
  EXPECT_TRUE(geom->getTriangleCount() == 2);

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getTriangles() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVerticeCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getTriangleCount() == 2);
}

TEST(TesseractGeometryUnit, Octree)  // NOLINT
{
  using T = tesseract_geometry::Octree;
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
  auto geom = std::make_shared<T>(pc, 0.01, tesseract_geometry::Octree::SubType::BOX, false);
  EXPECT_TRUE(geom->getOctree() != nullptr);
  EXPECT_TRUE(geom->getSubType() == tesseract_geometry::Octree::SubType::BOX);

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getOctree() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getSubType() == tesseract_geometry::Octree::SubType::BOX);
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

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL

TEST(TesseractGeometryUnit, LoadMeshWithMaterialGltf2Unit)  // NOLINT
{
  using namespace tesseract_geometry;

  std::string mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/tesseract_material_mesh.glb";
  std::vector<Mesh::Ptr> meshes =
      createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true, true, true, true);
  ASSERT_TRUE(meshes.size() == 4);

  auto& mesh0 = meshes[0];
  auto& mesh1 = meshes[1];
  auto& mesh2 = meshes[2];
  auto& mesh3 = meshes[3];

  EXPECT_EQ(mesh0->getTriangleCount(), 34);
  EXPECT_EQ(mesh0->getVerticeCount(), 68);
  EXPECT_EQ(mesh1->getTriangleCount(), 15);
  EXPECT_EQ(mesh1->getVerticeCount(), 17);
  EXPECT_EQ(mesh2->getTriangleCount(), 15);
  EXPECT_EQ(mesh2->getVerticeCount(), 17);
  EXPECT_EQ(mesh3->getTriangleCount(), 2);
  EXPECT_EQ(mesh3->getVerticeCount(), 4);

  auto mesh0_normals = mesh0->getNormals();
  ASSERT_TRUE(mesh0_normals != nullptr);
  EXPECT_EQ(mesh0_normals->size(), 68);
  auto mesh1_normals = mesh1->getNormals();
  ASSERT_TRUE(mesh1_normals != nullptr);
  EXPECT_EQ(mesh1_normals->size(), 17);
  auto mesh2_normals = mesh2->getNormals();
  ASSERT_TRUE(mesh2_normals != nullptr);
  EXPECT_EQ(mesh2_normals->size(), 17);
  auto mesh3_normals = mesh3->getNormals();
  ASSERT_TRUE(mesh3_normals != nullptr);
  EXPECT_EQ(mesh3_normals->size(), 4);

  auto mesh0_material = mesh0->getMaterial();
  EXPECT_TRUE(mesh0_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.7, 0.7, 0.7, 1), 0.01));
  EXPECT_NEAR(mesh0_material->getMetallicFactor(), 0.0, 0.01);
  EXPECT_NEAR(mesh0_material->getRoughnessFactor(), 0.5, 0.01);
  EXPECT_TRUE(mesh0_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  auto mesh1_material = mesh1->getMaterial();
  EXPECT_TRUE(mesh1_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.8, 0, 0, 1), 0.01));
  EXPECT_NEAR(mesh1_material->getMetallicFactor(), 0.8, 0.01);
  EXPECT_NEAR(mesh1_material->getRoughnessFactor(), 0.1, 0.01);
  EXPECT_TRUE(mesh1_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  auto mesh2_material = mesh2->getMaterial();
  EXPECT_TRUE(mesh2_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.05, 0.8, 0.05, 1), 0.01));
  EXPECT_NEAR(mesh2_material->getMetallicFactor(), 0.9, 0.01);
  EXPECT_NEAR(mesh2_material->getRoughnessFactor(), 0.7, 0.01);
  EXPECT_TRUE(mesh2_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0.1, 0.1, 0.5, 1), 0.01));

  auto mesh3_material = mesh3->getMaterial();
  EXPECT_TRUE(mesh3_material->getBaseColorFactor().isApprox(Eigen::Vector4d(1, 1, 1, 1), 0.01));
  EXPECT_NEAR(mesh3_material->getMetallicFactor(), 0, 0.01);
  EXPECT_NEAR(mesh3_material->getRoughnessFactor(), 0.5, 0.01);
  EXPECT_TRUE(mesh3_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  EXPECT_TRUE(mesh0->getTextures() == nullptr);
  EXPECT_TRUE(mesh1->getTextures() == nullptr);
  EXPECT_TRUE(mesh2->getTextures() == nullptr);

  ASSERT_TRUE(mesh3->getTextures() != nullptr);
  ASSERT_EQ(mesh3->getTextures()->size(), 1);

  auto texture = mesh3->getTextures()->at(0);
  EXPECT_EQ(texture->getTextureImage()->getResourceContents().size(), 38212);
  EXPECT_EQ(texture->getUVs()->size(), 4);
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
