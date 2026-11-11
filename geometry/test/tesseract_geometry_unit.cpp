#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>

extern "C" {
#include <tinyvdb_io.h>
#include <tinyvdb_nanovdb.h>
#include <tinyvdb_sparse_tree.h>
}
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/mesh_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <assimp/version.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/geometry/conversions.h>
#include <tesseract/geometry/utils.h>
#include <tesseract/geometry/impl/octree_utils.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/common/utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/ply_io.h>
#include <cstring>

static constexpr double BIG_TOL = 1e6;

// Helper macro to check a mesh
void checkTriangleMesh(const tesseract::geometry::Mesh& mesh, std::size_t exp_v, std::size_t exp_t)
{
  const auto& verts = mesh.getVertices();
  const auto& norms = mesh.getNormals();
  const auto& faces = mesh.getFaces();
  EXPECT_EQ(verts->size(), exp_v) << " vertex count";
  EXPECT_EQ(norms->size(), exp_v) << " normal count";
  EXPECT_EQ(faces->size(), exp_t) << " triangle index count";
  /* every face record should start with a '3' */
  for (int i = 0; i < faces->size(); i += 4)
  {
    EXPECT_EQ((*faces)[i], 3) << " face begins with 3";
  }
  /* normals should be unit length */
  for (const auto& n : *norms)
  {
    EXPECT_NEAR(n.norm(), 1.0, 1e-6) << " non‐unit normal";
  }
}

TEST(TesseractGeometryUnit, ToTriangleMeshSphere)  // NOLINT
{
  tesseract::geometry::Sphere s(1.0);
  auto mesh = toTriangleMesh(s, BIG_TOL);
  // lat=3,lon=3 ⇒ verts=(3−1)*3+2=8, tris=12⇒face‐entries=12*4=48
  checkTriangleMesh(*mesh, 8, 48);
}

TEST(TesseractGeometryUnit, ToTriangleMeshBox)  // NOLINT
{
  tesseract::geometry::Box b(2.0, 4.0, 6.0);
  auto mesh = toTriangleMesh(b, BIG_TOL);
  // always 8 corners, 12 tris ⇒ 8 verts, 48 face‐entries
  checkTriangleMesh(*mesh, 8, 48);
}

TEST(TesseractGeometryUnit, ToTriangleMeshCylinder)  // NOLINT
{
  tesseract::geometry::Cylinder c(1.0, 2.0);
  auto mesh = toTriangleMesh(c, BIG_TOL);
  // radial=3 ⇒ verts=2 + 2*3 = 8, tris=12⇒48 face‐entries
  checkTriangleMesh(*mesh, 8, 48);
}

TEST(TesseractGeometryUnit, ToTriangleMeshCone)  // NOLINT
{
  tesseract::geometry::Cone c(1.0, 2.0);
  auto mesh = toTriangleMesh(c, BIG_TOL);
  // radial=3 ⇒ verts=2 + 3 = 5, tris=6 ⇒ 6*4=24 face‐entries
  checkTriangleMesh(*mesh, 5, 24);
}

TEST(TesseractGeometryUnit, ToTriangleMeshCapsule)  // NOLINT
{
  tesseract::geometry::Capsule c(1.0, 2.0);
  auto mesh = toTriangleMesh(c, BIG_TOL);
  // hemi_rows=4/2=2, cyl=1, lon=6 ⇒ total_rows=2*2+1+2=7, cols=7
  // verts=7*7=49, tris=(total_rows−1)*lon*2=72⇒*4=288 face‐entries
  checkTriangleMesh(*mesh, 49, 288);
}

TEST(TesseractGeometryUnit, ToTriangleMeshFailure)  // NOLINT
{
  tesseract::geometry::Plane p(1.0, 2.0, 3.0, 4.0);
  EXPECT_ANY_THROW(toTriangleMesh(p, BIG_TOL));  // NOLINT
}

TEST(TesseractGeometryUnit, Instantiation)  // NOLINT
{
  auto vertices = std::make_shared<const tesseract::common::VectorVector3d>();
  auto faces = std::make_shared<const Eigen::VectorXi>();

  auto box = std::make_shared<tesseract::geometry::Box>(1, 1, 1);
  auto cone = std::make_shared<tesseract::geometry::Cone>(1, 1);
  auto cylinder = std::make_shared<tesseract::geometry::Cylinder>(1, 1);
  auto capsule = std::make_shared<tesseract::geometry::Capsule>(1, 1);
  auto plane = std::make_shared<tesseract::geometry::Plane>(1, 1, 1, 1);
  auto sphere = std::make_shared<tesseract::geometry::Sphere>(1);
  auto convex_mesh = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces);
  auto mesh = std::make_shared<tesseract::geometry::Mesh>(vertices, faces);
  auto octree = std::make_shared<tesseract::geometry::Octree>(nullptr, tesseract::geometry::OctreeSubType::BOX);
  auto compound_mesh = std::make_shared<tesseract::geometry::CompoundMesh>(
      std::vector<std::shared_ptr<tesseract::geometry::PolygonMesh>>{
          std::make_shared<tesseract::geometry::Mesh>(vertices, faces),
          std::make_shared<tesseract::geometry::Mesh>(vertices, faces) });

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
  auto co = tesseract::geometry::createOctree(pc, 0.01, false);
  auto octree_pc =
      std::make_shared<tesseract::geometry::Octree>(std::move(co), tesseract::geometry::OctreeSubType::BOX, false);
}

TEST(TesseractGeometryUnit, Box)  // NOLINT
{
  using T = tesseract::geometry::Box;
  auto geom = std::make_shared<T>(1, 1, 1);
  EXPECT_NEAR(geom->getX(), 1, 1e-5);
  EXPECT_NEAR(geom->getY(), 1, 1e-5);
  EXPECT_NEAR(geom->getZ(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::BOX);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getX(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getY(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getZ(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::BOX);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Box(2, 1, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Box(1, 2, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Box(1, 1, 2)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Cone(1, 1)));
}

TEST(TesseractGeometryUnit, Cone)  // NOLINT
{
  using T = tesseract::geometry::Cone;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::CONE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::CONE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Cone(2, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Cone(1, 2)));
}

TEST(TesseractGeometryUnit, Cylinder)  // NOLINT
{
  using T = tesseract::geometry::Cylinder;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::CYLINDER);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::CYLINDER);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Cylinder(2, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Cylinder(1, 2)));
}

TEST(TesseractGeometryUnit, Capsule)  // NOLINT
{
  using T = tesseract::geometry::Capsule;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::CAPSULE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::CAPSULE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Capsule(2, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Capsule(1, 2)));
}

TEST(TesseractGeometryUnit, Sphere)  // NOLINT
{
  using T = tesseract::geometry::Sphere;
  auto geom = std::make_shared<T>(1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::SPHERE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::SPHERE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Sphere(2)));
}

TEST(TesseractGeometryUnit, Plane)  // NOLINT
{
  using T = tesseract::geometry::Plane;
  auto geom = std::make_shared<T>(1, 1, 1, 1);
  EXPECT_NEAR(geom->getA(), 1, 1e-5);
  EXPECT_NEAR(geom->getB(), 1, 1e-5);
  EXPECT_NEAR(geom->getC(), 1, 1e-5);
  EXPECT_NEAR(geom->getD(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::PLANE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getA(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getB(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getC(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getD(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::PLANE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Plane(2, 1, 1, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Plane(1, 2, 1, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Plane(1, 1, 2, 1)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::Plane(1, 1, 1, 2)));
}

TEST(TesseractGeometryUnit, PolygonMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

  using T = tesseract::geometry::PolygonMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVertexCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 1);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::POLYGON_MESH);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 1);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::POLYGON_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(
      *geom,
      tesseract::geometry::PolygonMesh(std::make_shared<tesseract::common::VectorVector3d>(),
                                       std::make_shared<Eigen::VectorXi>())));
}

TEST(TesseractGeometryUnit, MeshParserVertexColors)  // NOLINT
{
  // The vertex_colors flag reaches the loaded mesh; the normals arm is the control
  tesseract::common::VectorVector3d verts{ { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
  std::vector<Eigen::Vector3i> colors{ Eigen::Vector3i{ 255, 0, 0 },
                                       Eigen::Vector3i{ 0, 255, 0 },
                                       Eigen::Vector3i{ 0, 0, 255 } };
  Eigen::VectorXi faces(4);
  faces << 3, 0, 1, 2;

  const std::string path = tesseract::common::getTempPath() + "vertex_color_mesh.ply";
  ASSERT_TRUE(tesseract::common::writeSimplePlyFile(path, verts, colors, faces, 1));

  auto resource = std::make_shared<tesseract::common::SimpleLocatedResource>(path, path);

  auto with_colors = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
      resource, Eigen::Vector3d(1, 1, 1), true, true, true, true, true);
  ASSERT_EQ(with_colors.size(), 1U);
  ASSERT_TRUE(with_colors.front()->getVertexColors() != nullptr);
  ASSERT_EQ(with_colors.front()->getVertexColors()->size(), verts.size());
  for (std::size_t i = 0; i < colors.size(); ++i)
  {
    const Eigen::Vector4d& c = with_colors.front()->getVertexColors()->at(i);
    EXPECT_NEAR(c(0), colors[i](0) / 255.0, 1e-3);
    EXPECT_NEAR(c(1), colors[i](1) / 255.0, 1e-3);
    EXPECT_NEAR(c(2), colors[i](2) / 255.0, 1e-3);
  }

  auto without_colors = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
      resource, Eigen::Vector3d(1, 1, 1), true, true, true, false, true);
  ASSERT_EQ(without_colors.size(), 1U);
  EXPECT_TRUE(without_colors.front()->getVertexColors() == nullptr);
}

/** @brief The optional data a mesh carries in addition to its vertices and faces */
struct MeshDecorations
{
  std::shared_ptr<const tesseract::common::VectorVector3d> normals;
  std::shared_ptr<const tesseract::common::VectorVector4d> vertex_colors;
  std::shared_ptr<tesseract::geometry::MeshMaterial> material;
  std::shared_ptr<const std::vector<tesseract::geometry::MeshTexture::Ptr>> textures;
};

/** @brief Build a full set of decorations for a mesh of @p vertex_count vertices */
MeshDecorations makeMeshDecorations(std::size_t vertex_count)
{
  auto uvs = std::make_shared<tesseract::common::VectorVector2d>(1, Eigen::Vector2d(0, 0));
  return { std::make_shared<tesseract::common::VectorVector3d>(vertex_count, Eigen::Vector3d(0, 0, 1)),
           std::make_shared<tesseract::common::VectorVector4d>(vertex_count, Eigen::Vector4d(1, 0, 0, 1)),
           std::make_shared<tesseract::geometry::MeshMaterial>(
               Eigen::Vector4d(0.1, 0.2, 0.3, 1.0), 0.25, 0.75, Eigen::Vector4d(0.4, 0.5, 0.6, 1.0)),
           std::make_shared<std::vector<tesseract::geometry::MeshTexture::Ptr>>(
               1, std::make_shared<tesseract::geometry::MeshTexture>(nullptr, uvs)) };
}

/**
 * @brief Check @p clone carries the decorations it was built from
 * @details The containers are shared with the source, but the material is deep copied, so the clone must hold an
 * equal material in a different object.
 */
void expectDecorationsCloned(const tesseract::geometry::PolygonMesh& clone, const MeshDecorations& source)
{
  EXPECT_EQ(clone.getNormals(), source.normals);
  EXPECT_EQ(clone.getVertexColors(), source.vertex_colors);
  EXPECT_EQ(clone.getTextures(), source.textures);

  const auto material = clone.getMaterial();
  ASSERT_TRUE(material != nullptr);
  EXPECT_NE(material.get(), source.material.get());
  EXPECT_TRUE(material->getBaseColorFactor().isApprox(source.material->getBaseColorFactor()));
  EXPECT_TRUE(material->getEmissiveFactor().isApprox(source.material->getEmissiveFactor()));
  EXPECT_DOUBLE_EQ(material->getMetallicFactor(), source.material->getMetallicFactor());
  EXPECT_DOUBLE_EQ(material->getRoughnessFactor(), source.material->getRoughnessFactor());
}

TEST(TesseractGeometryUnit, PolygonMeshCloneAndEquality)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(4);
  (*faces) << 3, 0, 1, 2;

  auto deco = makeMeshDecorations(3);

  using T = tesseract::geometry::PolygonMesh;
  auto geom = std::make_shared<T>(vertices,
                                  faces,
                                  1,
                                  nullptr,
                                  Eigen::Vector3d(1, 1, 1),
                                  deco.normals,
                                  deco.vertex_colors,
                                  deco.material,
                                  deco.textures);

  auto clone = std::static_pointer_cast<T>(geom->clone());
  expectDecorationsCloned(*clone, deco);
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *clone));

  // Equality looks at the geometry, not just the counts and scale
  auto other_vertices = std::make_shared<tesseract::common::VectorVector3d>();
  other_vertices->emplace_back(5, 5, 0);
  other_vertices->emplace_back(5, -5, 0);
  other_vertices->emplace_back(-5, -5, 0);
  T other(other_vertices, faces, 1);
  other.setUUID(geom->getUUID());
  EXPECT_TRUE(*geom != other);
}

TEST(TesseractGeometryUnit, ConvexMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

  auto deco = makeMeshDecorations(4);

  using T = tesseract::geometry::ConvexMesh;
  auto geom = std::make_shared<T>(vertices,
                                  faces,
                                  nullptr,
                                  Eigen::Vector3d(1, 1, 1),
                                  deco.normals,
                                  deco.vertex_colors,
                                  deco.material,
                                  deco.textures);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVertexCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 1);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::CONVEX_MESH);
  EXPECT_EQ(geom->getCreationMethod(), tesseract::geometry::ConvexMesh::CreationMethod::DEFAULT);
  geom->setCreationMethod(tesseract::geometry::ConvexMesh::CreationMethod::CONVERTED);
  EXPECT_EQ(geom->getCreationMethod(), tesseract::geometry::ConvexMesh::CreationMethod::CONVERTED);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto clone = std::static_pointer_cast<T>(geom->clone());
  EXPECT_TRUE(clone->getVertices() != nullptr);
  EXPECT_TRUE(clone->getFaces() != nullptr);
  EXPECT_TRUE(clone->getVertexCount() == 4);
  EXPECT_TRUE(clone->getFaceCount() == 1);
  EXPECT_EQ(clone->getType(), tesseract::geometry::GeometryType::CONVEX_MESH);
  EXPECT_EQ(clone->getCreationMethod(), tesseract::geometry::ConvexMesh::CreationMethod::CONVERTED);
  expectDecorationsCloned(*clone, deco);
  EXPECT_FALSE(clone->getUUID().is_nil());
  EXPECT_NE(clone->getUUID(), geom->getUUID());

  geom->setUUID(clone->getUUID());
  EXPECT_EQ(clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(
      *geom,
      tesseract::geometry::ConvexMesh(std::make_shared<tesseract::common::VectorVector3d>(),
                                      std::make_shared<Eigen::VectorXi>())));
}

TEST(TesseractGeometryUnit, CompoundConvexMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

  using T = tesseract::geometry::ConvexMesh;
  auto sub_geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(sub_geom->getVertices() != nullptr);
  EXPECT_TRUE(sub_geom->getFaces() != nullptr);
  EXPECT_TRUE(sub_geom->getVertexCount() == 4);
  EXPECT_TRUE(sub_geom->getFaceCount() == 1);
  EXPECT_EQ(sub_geom->getType(), tesseract::geometry::GeometryType::CONVEX_MESH);
  EXPECT_FALSE(sub_geom->getUUID().is_nil());

  std::vector<tesseract::geometry::PolygonMesh::Ptr> poly_meshes;
  poly_meshes.push_back(sub_geom);
  poly_meshes.push_back(sub_geom);
  poly_meshes.push_back(sub_geom);

  auto geom = std::make_shared<tesseract::geometry::CompoundMesh>(poly_meshes);
  EXPECT_EQ(geom->getMeshes().size(), 3);
  EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);

  auto geom_clone = std::static_pointer_cast<tesseract::geometry::CompoundMesh>(geom->clone());
  EXPECT_EQ(geom_clone->getMeshes().size(), 3);
  EXPECT_EQ(geom_clone->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(
      tesseract::common::almostEqualRelativeAndAbs(geom_clone->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::CompoundMesh()));

  {  // Test convex hull constructors
    std::vector<tesseract::geometry::ConvexMesh::Ptr> convex_meshes;
    convex_meshes.push_back(sub_geom);
    convex_meshes.push_back(sub_geom);
    convex_meshes.push_back(sub_geom);

    auto geom = std::make_shared<tesseract::geometry::CompoundMesh>(convex_meshes);
    EXPECT_EQ(geom->getMeshes().size(), 3);
    EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
    EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);
  }

  {  // Test convex hull constructors
    auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

    using T = tesseract::geometry::Mesh;
    auto sub_geom = std::make_shared<T>(vertices, faces);
    EXPECT_TRUE(sub_geom->getVertices() != nullptr);
    EXPECT_TRUE(sub_geom->getFaces() != nullptr);
    EXPECT_TRUE(sub_geom->getVertexCount() == 4);
    EXPECT_TRUE(sub_geom->getFaceCount() == 2);
    EXPECT_EQ(sub_geom->getType(), tesseract::geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    std::vector<tesseract::geometry::Mesh::Ptr> meshes;
    meshes.push_back(sub_geom);
    meshes.push_back(sub_geom);
    meshes.push_back(sub_geom);

    auto geom = std::make_shared<tesseract::geometry::CompoundMesh>(meshes);
    EXPECT_EQ(geom->getMeshes().size(), 3);
    EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
    EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);
  }
}

TEST(TesseractGeometryUnit, Mesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

  using T = tesseract::geometry::Mesh;

  {
    auto geom = std::make_shared<T>(vertices, faces);
    EXPECT_TRUE(geom->getVertices() != nullptr);
    EXPECT_TRUE(geom->getFaces() != nullptr);
    EXPECT_TRUE(geom->getVertexCount() == 4);
    EXPECT_TRUE(geom->getFaceCount() == 2);
    EXPECT_TRUE(geom->getMaterial() == nullptr);
    EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    auto geom_clone = geom->clone();
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 2);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getMaterial() == nullptr);
    EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::MESH);
    EXPECT_FALSE(geom_clone->getUUID().is_nil());
    EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

    geom->setUUID(geom_clone->getUUID());
    EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

    // Test isIdentical
    EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
    EXPECT_FALSE(tesseract::geometry::isIdentical(
        *geom,
        tesseract::geometry::Mesh(std::make_shared<tesseract::common::VectorVector3d>(),
                                  std::make_shared<Eigen::VectorXi>())));
  }

  {
    auto deco = makeMeshDecorations(4);
    auto geom = std::make_shared<T>(vertices,
                                    faces,
                                    nullptr,
                                    Eigen::Vector3d(1, 1, 1),
                                    deco.normals,
                                    deco.vertex_colors,
                                    deco.material,
                                    deco.textures);
    EXPECT_TRUE(geom->getVertices() != nullptr);
    EXPECT_TRUE(geom->getFaces() != nullptr);
    EXPECT_TRUE(geom->getVertexCount() == 4);
    EXPECT_TRUE(geom->getFaceCount() == 2);
    EXPECT_TRUE(geom->getMaterial() != nullptr);
    EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    auto clone = std::static_pointer_cast<T>(geom->clone());
    EXPECT_TRUE(clone->getVertices() != nullptr);
    EXPECT_TRUE(clone->getFaces() != nullptr);
    EXPECT_TRUE(clone->getVertexCount() == 4);
    EXPECT_TRUE(clone->getFaceCount() == 2);
    expectDecorationsCloned(*clone, deco);
    EXPECT_EQ(clone->getType(), tesseract::geometry::GeometryType::MESH);
    EXPECT_FALSE(clone->getUUID().is_nil());
    EXPECT_NE(clone->getUUID(), geom->getUUID());

    geom->setUUID(clone->getUUID());
    EXPECT_EQ(clone->getUUID(), geom->getUUID());

    // Test isIdentical
    EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *clone));
    EXPECT_FALSE(tesseract::geometry::isIdentical(
        *geom,
        tesseract::geometry::Mesh(std::make_shared<tesseract::common::VectorVector3d>(),
                                  std::make_shared<Eigen::VectorXi>())));
  }
}

TEST(TesseractGeometryUnit, CompoundMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
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

  using T = tesseract::geometry::Mesh;

  auto sub_geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(sub_geom->getVertices() != nullptr);
  EXPECT_TRUE(sub_geom->getFaces() != nullptr);
  EXPECT_TRUE(sub_geom->getVertexCount() == 4);
  EXPECT_TRUE(sub_geom->getFaceCount() == 2);
  EXPECT_TRUE(sub_geom->getMaterial() == nullptr);
  EXPECT_EQ(sub_geom->getType(), tesseract::geometry::GeometryType::MESH);
  EXPECT_FALSE(sub_geom->getUUID().is_nil());

  std::vector<tesseract::geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(sub_geom);
  meshes.push_back(sub_geom);
  meshes.push_back(sub_geom);

  auto geom = std::make_shared<tesseract::geometry::CompoundMesh>(meshes);
  EXPECT_EQ(geom->getMeshes().size(), 3);
  EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = std::static_pointer_cast<tesseract::geometry::CompoundMesh>(geom->clone());
  EXPECT_EQ(geom_clone->getMeshes().size(), 3);
  EXPECT_EQ(geom_clone->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(
      tesseract::common::almostEqualRelativeAndAbs(geom_clone->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test Failure
  meshes.clear();
  meshes.push_back(sub_geom);
  EXPECT_ANY_THROW(std::make_shared<tesseract::geometry::CompoundMesh>(meshes));  // NOLINT

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::CompoundMesh()));

  meshes.push_back(sub_geom);
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, tesseract::geometry::CompoundMesh(meshes)));
}

TEST(TesseractGeometryUnit, SignedDistanceField)  // NOLINT
{
  using T = tesseract::geometry::SignedDistanceField;
  const Eigen::AlignedBox3d domain(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  const Eigen::Vector3i dims(2, 2, 2);
  const std::vector<double> distances{ -0.5, -0.4, -0.3, -0.2, 0.1, 0.2, 0.3, 0.4 };
  const Eigen::Vector3d scale(1.0, 2.0, 3.0);

  auto geom = std::make_shared<T>(domain, dims, distances, scale);
  EXPECT_EQ(geom->getDistances(), distances);
  EXPECT_EQ(geom->getDimensions(), dims);
  EXPECT_TRUE(geom->getDomain().min().isApprox(domain.min(), 1e-9));
  EXPECT_TRUE(geom->getDomain().max().isApprox(domain.max(), 1e-9));
  EXPECT_TRUE(geom->getScale().isApprox(scale, 1e-5));
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD);
  EXPECT_FALSE(geom->getUUID().is_nil());

  // Trilinear sampling: a corner returns its stored value exactly; the center is the 8-corner mean.
  EXPECT_NEAR(geom->getDistance(Eigen::Vector3d(-1, -1, -1)), distances.front(), 1e-9);
  const double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<double>(distances.size());
  EXPECT_NEAR(geom->getDistance(Eigen::Vector3d(0, 0, 0)), mean, 1e-9);
  // Points outside the domain clamp to the boundary.
  EXPECT_NEAR(geom->getDistance(Eigen::Vector3d(-5, -5, -5)), distances.front(), 1e-9);

  // Invalid construction is rejected.
  EXPECT_ANY_THROW(T(domain, Eigen::Vector3i(1, 2, 2), std::vector<double>(4, 0.0)));          // NOLINT dim < 2
  EXPECT_ANY_THROW(T(domain, dims, std::vector<double>(7, 0.0)));                              // NOLINT wrong count
  EXPECT_ANY_THROW(T(Eigen::AlignedBox3d(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 1)),  // NOLINT degenerate
                     dims,
                     distances));
  EXPECT_ANY_THROW(T(domain, dims, distances, Eigen::Vector3d(0, 1, 1)));   // NOLINT zero scale
  EXPECT_ANY_THROW(T(domain, dims, distances, Eigen::Vector3d(-1, 1, 1)));  // NOLINT negative scale
  EXPECT_ANY_THROW(
      T(domain, dims, distances, Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 1, 1)));  // NOLINT

  auto geom_clone = geom->clone();
  EXPECT_EQ(std::static_pointer_cast<T>(geom_clone)->getDistances(), distances);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getScale().isApprox(scale, 1e-5));
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  const std::vector<double> other(8, 9.0);

  // operator== / operator!=
  EXPECT_TRUE(*geom == *std::static_pointer_cast<T>(geom_clone));
  EXPECT_FALSE(*geom != *std::static_pointer_cast<T>(geom_clone));
  EXPECT_TRUE(*geom != T(domain, dims, other, scale));

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, T(domain, dims, other, scale)));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*geom, T(domain, dims, distances, Eigen::Vector3d(1, 1, 1))));

  // The grid round-trips through standard OpenVDB serialization.
  const std::vector<std::uint8_t> vdb = tesseract::geometry::writeSignedDistanceFieldVDB(*geom);
  auto geom_rt = tesseract::geometry::readSignedDistanceFieldVDB(vdb.data(), vdb.size(), scale);
  ASSERT_EQ(geom_rt->getDistances().size(), distances.size());
  for (std::size_t index = 0; index < distances.size(); ++index)
    EXPECT_NEAR(geom_rt->getDistances()[index], distances[index], 1e-6);
  EXPECT_EQ(geom_rt->getDimensions(), dims);
  EXPECT_TRUE(geom_rt->getDomain().min().isApprox(domain.min(), 1e-9));
  EXPECT_TRUE(geom_rt->getDomain().max().isApprox(domain.max(), 1e-9));
  EXPECT_TRUE(geom_rt->getScale().isApprox(scale, 1e-5));

  // The grid also round-trips through standard NanoVDB serialization.
  const std::vector<std::uint8_t> nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(*geom);
  auto geom_nvdb_rt = tesseract::geometry::readSignedDistanceFieldNVDB(nvdb.data(), nvdb.size(), scale);
  ASSERT_EQ(geom_nvdb_rt->getDistances().size(), distances.size());
  for (std::size_t index = 0; index < distances.size(); ++index)
    EXPECT_NEAR(geom_nvdb_rt->getDistances()[index], distances[index], 1e-6);
  EXPECT_EQ(geom_nvdb_rt->getDimensions(), dims);
  EXPECT_TRUE(geom_nvdb_rt->getDomain().min().isApprox(domain.min(), 1e-9));
  EXPECT_TRUE(geom_nvdb_rt->getDomain().max().isApprox(domain.max(), 1e-9));
  EXPECT_TRUE(geom_nvdb_rt->getScale().isApprox(scale, 1e-5));

  // A malformed VDB is rejected.
  EXPECT_ANY_THROW(tesseract::geometry::readSignedDistanceFieldVDB(std::vector<std::uint8_t>{ 0, 1, 2, 3 }));  // NOLINT
  EXPECT_ANY_THROW(
      tesseract::geometry::readSignedDistanceFieldNVDB(std::vector<std::uint8_t>{ 0, 1, 2, 3 }));  // NOLINT
}

TEST(TesseractGeometryUnit, SignedDistanceFieldBloscNVDB)  // NOLINT
{
  // Written by fVDB (Grid.save_nanovdb(..., compressed=True)), as real NanoVDB producers write them: BLOSC-compressed,
  // with the grid name's terminator counted in its size. A 10x9x8 dense grid of voxel 0.05 at origin (0.1, -0.2, 0.3)
  // and index offset (-3, -2, -1), holding the signed distance to a sphere of radius 0.15 centred at (0.12, -0.17,
  // 0.32).
  tesseract::common::GeneralResourceLocator locator;
  const std::vector<std::uint8_t> nvdb =
      locator.locateResource("package://tesseract/support/sdf/sphere_blosc.nvdb")->getResourceContents();
  const Eigen::Vector3d scale(2, 2, 2);

#ifdef TESSERACT_TINYVDB_HAS_BLOSC
  auto sdf = tesseract::geometry::readSignedDistanceFieldNVDB(nvdb, scale);
  EXPECT_EQ(sdf->getDimensions(), Eigen::Vector3i(10, 9, 8));
  EXPECT_TRUE(sdf->getDomain().min().isApprox(Eigen::Vector3d(-0.05, -0.3, 0.25), 1e-6));
  EXPECT_TRUE(sdf->getDomain().max().isApprox(Eigen::Vector3d(0.4, 0.1, 0.6), 1e-6));
  EXPECT_TRUE(sdf->getScale().isApprox(scale));

  const Eigen::Vector3d centre(0.12, -0.17, 0.32);
  const auto& distances = sdf->getDistances();
  ASSERT_EQ(distances.size(), 720);
  for (int k = 0; k < 8; ++k)
    for (int j = 0; j < 9; ++j)
      for (int i = 0; i < 10; ++i)
      {
        const Eigen::Vector3d p = sdf->getDomain().min() + 0.05 * Eigen::Vector3d(i, j, k);
        EXPECT_NEAR(distances[static_cast<std::size_t>(i + (10 * (j + (9 * k))))], (p - centre).norm() - 0.15, 1e-6);
      }
#else
  // Without Blosc the grid cannot be decoded, and must be rejected rather than read as garbage.
  EXPECT_THROW(tesseract::geometry::readSignedDistanceFieldNVDB(nvdb, scale), std::runtime_error);  // NOLINT
#endif
}

TEST(TesseractGeometryUnit, SignedDistanceFieldSampler)  // NOLINT
{
  // Discretize a sphere distance field on a dense grid and verify the stored samples and interpolation.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  const Eigen::Vector3d dmin(-1, -1, -1);
  const Eigen::Vector3d dmax(1, 1, 1);
  const Eigen::Vector3i dims(9, 9, 9);
  auto geom = tesseract::geometry::createDiscreteSignedDistanceField(sphere, dmin, dmax, dims);

  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD);
  EXPECT_EQ(geom->getDimensions(), dims);
  EXPECT_EQ(geom->getDistances().size(), 9U * 9U * 9U);

  // The first sample sits at the domain minimum corner, so it equals sphere(dmin).
  EXPECT_NEAR(geom->getDistances().front(), std::sqrt(3.0) - 0.5, 1e-9);
  // The field interpolates back to roughly the true distance at the center.
  EXPECT_NEAR(geom->getDistance(Eigen::Vector3d(0, 0, 0)), -0.5, 0.05);

  // Invalid inputs are rejected.
  EXPECT_ANY_THROW(tesseract::geometry::createDiscreteSignedDistanceField(  // NOLINT degenerate domain
      sphere,
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 1, 1),
      dims));
  EXPECT_ANY_THROW(tesseract::geometry::createDiscreteSignedDistanceField(  // NOLINT dimension < 2
      sphere,
      dmin,
      dmax,
      Eigen::Vector3i(1, 2, 2)));
}

TEST(TesseractGeometryUnit, SignedDistanceFieldSamplerBatched)  // NOLINT
{
  const Eigen::Vector3d dmin(-1, -1, -1);
  const Eigen::Vector3d dmax(1, 1, 1);
  const Eigen::Vector3i dims(4, 4, 4);
  const Eigen::Vector3d scale(1.0, 2.0, 3.0);

  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  const tesseract::geometry::BatchedSignedDistanceFunction sphere_batched =
      [&sphere](const std::vector<Eigen::Vector3d>& pts) {
        std::vector<double> out;
        out.reserve(pts.size());
        for (const auto& p : pts)
          out.push_back(sphere(p));
        return out;
      };

  // Per-point and batched sampling must produce identical grids (same nodes, same order).
  auto geom = tesseract::geometry::createDiscreteSignedDistanceField(sphere, dmin, dmax, dims, scale);
  auto geom_batched = tesseract::geometry::createDiscreteSignedDistanceField(sphere_batched, dmin, dmax, dims, scale);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD);
  EXPECT_EQ(geom->getDistances(), geom_batched->getDistances());
  EXPECT_EQ(geom->getDimensions(), dims);
  EXPECT_TRUE(geom->getScale().isApprox(scale, 1e-5));

  // A batched function that returns the wrong number of values is rejected.
  const tesseract::geometry::BatchedSignedDistanceFunction wrong_count = [](const std::vector<Eigen::Vector3d>&) {
    return std::vector<double>{ 1.0 };
  };
  EXPECT_ANY_THROW(tesseract::geometry::createDiscreteSignedDistanceField(wrong_count, dmin, dmax, dims));  // NOLINT
}

TEST(TesseractGeometryUnit, SignedDistanceFieldLazy)  // NOLINT
{
  const Eigen::Vector3d dmin(-1, -1, -1);
  const Eigen::Vector3d dmax(1, 1, 1);
  const Eigen::Vector3i dims(8, 8, 8);
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };

  // Lazy field: not discretized up front; getDistance hits the sampler directly (exact) and does not discretize.
  auto lazy = tesseract::geometry::createSignedDistanceField(sphere, dmin, dmax, dims);
  EXPECT_FALSE(lazy->isDiscretized());
  EXPECT_NEAR(lazy->getDistance(Eigen::Vector3d(0, 0, 0)), -0.5, 1e-12);
  EXPECT_FALSE(lazy->isDiscretized());

  // Explicit discretize materializes the grid.
  lazy->discretize();
  EXPECT_TRUE(lazy->isDiscretized());
  EXPECT_EQ(lazy->getDistances().size(), 8U * 8U * 8U);

  // Eager is pre-discretized.
  auto eager = tesseract::geometry::createDiscreteSignedDistanceField(sphere, dmin, dmax, dims);
  EXPECT_TRUE(eager->isDiscretized());

  // Serializing a lazy field to VDB discretizes it, and the round-trip yields a grid-backed field.
  auto lazy2 = tesseract::geometry::createSignedDistanceField(sphere, dmin, dmax, dims);
  EXPECT_FALSE(lazy2->isDiscretized());
  const std::vector<std::uint8_t> vdb = tesseract::geometry::writeSignedDistanceFieldVDB(*lazy2);
  EXPECT_TRUE(lazy2->isDiscretized());
  auto rt = tesseract::geometry::readSignedDistanceFieldVDB(vdb);
  EXPECT_TRUE(rt->isDiscretized());
  EXPECT_EQ(rt->getDimensions(), dims);
  ASSERT_EQ(rt->getDistances().size(), lazy2->getDistances().size());
  for (std::size_t index = 0; index < rt->getDistances().size(); ++index)
    EXPECT_NEAR(rt->getDistances()[index], lazy2->getDistances()[index], 1e-6);

  // Cloning a lazy field preserves the sampler (clone queries exactly, still not yet discretized).
  auto clone = std::static_pointer_cast<tesseract::geometry::SignedDistanceField>(lazy2->clone());
  EXPECT_NEAR(clone->getDistance(Eigen::Vector3d(0, 0, 0)), -0.5, 1e-12);
}

namespace
{
/**
 * @brief A bare NanoVDB grid buffer holding @p f at origin + voxel * (i, j, k) for each index in
 * [@p lo, @p hi] whose position passes @p active (all of them when null). Other voxels are inactive.
 */
std::vector<std::uint8_t> makeNanoVDBGrid(const tesseract::geometry::SignedDistanceFunction& f,
                                          const Eigen::Vector3d& origin,
                                          double voxel,
                                          const Eigen::Vector3i& lo,
                                          const Eigen::Vector3i& hi,
                                          const std::function<bool(const Eigen::Vector3d&)>& active = nullptr,
                                          float background = 0.0F)
{
  std::vector<tvdb_vec3i> coordinates;
  std::vector<float> values;
  for (int k = lo.z(); k <= hi.z(); ++k)
    for (int j = lo.y(); j <= hi.y(); ++j)
      for (int i = lo.x(); i <= hi.x(); ++i)
      {
        const Eigen::Vector3d p = origin + (voxel * Eigen::Vector3d(i, j, k));
        if (active && !active(p))
          continue;
        coordinates.push_back({ i, j, k });
        values.push_back(static_cast<float>(f(p)));
      }

  std::string grid_type = "Tree_float_5_4_3";
  tvdb_grid_t template_grid{};
  template_grid.descriptor.grid_type = grid_type.data();
  template_grid.tree.layout.num_levels = 4;
  template_grid.tree.layout.levels[0] = { TVDB_NODE_ROOT, TVDB_VALUE_FLOAT, 0 };
  template_grid.tree.layout.levels[1] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 5 };
  template_grid.tree.layout.levels[2] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 4 };
  template_grid.tree.layout.levels[3] = { TVDB_NODE_LEAF, TVDB_VALUE_FLOAT, 3 };
  template_grid.transform.type = TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE;
  for (int axis = 0; axis < 3; ++axis)
  {
    template_grid.transform.scale_values[axis] = voxel;
    template_grid.transform.voxel_size[axis] = voxel;
    template_grid.transform.translation[axis] = origin[axis];
  }

  tvdb_grid_t grid{};
  if (!tvdb_grid_from_sparse_typed_using_template(&template_grid,
                                                  coordinates.data(),
                                                  values.data(),
                                                  values.size(),
                                                  TVDB_VALUE_FLOAT,
                                                  &background,
                                                  "test",
                                                  &grid))
    throw std::runtime_error("makeNanoVDBGrid: failed to build the grid");
  std::uint8_t* data = nullptr;
  std::size_t size = 0;
  tvdb_error_t error{};
  const tvdb_status_t status = tvdb_grid_to_nanovdb_float(&grid, &data, &size, &error);
  tvdb_grid_destroy_owned(&grid);
  if (status != TVDB_OK)
    throw std::runtime_error("makeNanoVDBGrid: failed to convert the grid");
  std::vector<std::uint8_t> result(data, data + size);
  std::free(data);  // NOLINT(cppcoreguidelines-no-malloc, cppcoreguidelines-owning-memory)
  return result;
}

/** @brief Offset of the grid's own header in @p bytes: the last "NanoVDB" magic (a file's comes first). */
std::ptrdiff_t gridHeaderOffset(const std::vector<std::uint8_t>& bytes)
{
  const std::string magic = "NanoVDB";
  return std::find_end(bytes.begin(), bytes.end(), magic.begin(), magic.end()) - bytes.begin();
}

/** @brief @p grid with its NanoVDB grid class (GridData offset 632) set to @p grid_class (1: LevelSet). */
std::vector<std::uint8_t> withGridClass(std::vector<std::uint8_t> grid, std::uint32_t grid_class)
{
  std::memcpy(grid.data() + gridHeaderOffset(grid) + 632, &grid_class, sizeof(grid_class));
  return grid;
}

/** @brief @p grid renamed @p name (GridData offset 40, 256 bytes). */
std::vector<std::uint8_t> withGridName(std::vector<std::uint8_t> grid, const std::string& name)
{
  std::uint8_t* field = grid.data() + gridHeaderOffset(grid) + 40;
  std::fill(field, field + 256, 0);
  std::copy(name.begin(), name.end(), field);
  return grid;
}

const tesseract::geometry::SignedDistanceFunction SPHERE = [](const Eigen::Vector3d& p) { return p.norm() - 0.3; };

/**
 * @brief A grid-backed field (no sampler) holding @p f on @p n samples per axis over [-half, half]^3.
 * @details Values pass through float, as a NanoVDB grid stores them, so the two can be compared exactly.
 */
std::shared_ptr<tesseract::geometry::SignedDistanceField>
makeGridField(const tesseract::geometry::SignedDistanceFunction& f, double half, int n)
{
  std::vector<double> distances;
  const double step = 2.0 * half / (n - 1);
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        distances.push_back(
            static_cast<float>(f(Eigen::Vector3d(-half + (i * step), -half + (j * step), -half + (k * step)))));
  return std::make_shared<tesseract::geometry::SignedDistanceField>(
      Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-half), Eigen::Vector3d::Constant(half)),
      Eigen::Vector3i::Constant(n),
      std::move(distances));
}

/** @brief Central difference of the field's distance, for checking getGradient. */
Eigen::Vector3d finiteDifferenceGradient(const tesseract::geometry::SignedDistanceField& field,
                                         const Eigen::Vector3d& point,
                                         double step)
{
  Eigen::Vector3d gradient;
  for (Eigen::Index axis = 0; axis < 3; ++axis)
  {
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    offset[axis] = step;
    gradient[axis] = (field.getDistance(point + offset) - field.getDistance(point - offset)) / (2.0 * step);
  }
  return gradient;
}

/** @brief Deterministic points in [-extent, extent]^3, kept off cell boundaries of a grid with @p voxel spacing. */
std::vector<Eigen::Vector3d> probePoints(double extent, double voxel, std::size_t count)
{
  std::vector<Eigen::Vector3d> points;
  for (std::size_t n = 0; points.size() < count; ++n)
  {
    const auto t = static_cast<double>(n);
    Eigen::Vector3d p(std::sin((1.3 * t) + 0.1), std::sin((2.1 * t) + 0.7), std::sin((3.7 * t) + 1.3));
    p *= extent;
    const Eigen::Vector3d fraction = (p / voxel).array() - (p / voxel).array().floor();
    if ((fraction.array() < 0.05).any() || (fraction.array() > 0.95).any())
      continue;
    points.push_back(p);
  }
  return points;
}
}  // namespace

TEST(TesseractGeometryUnit, SignedDistanceFieldGradient)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  const double voxel = 0.05;
  auto dense = makeGridField(SPHERE, 0.5, 21);
  for (const Eigen::Vector3d& p : probePoints(0.45, voxel, 50))
  {
    EXPECT_TRUE(dense->getGradient(p).isApprox(finiteDifferenceGradient(*dense, p, 1e-6), 1e-5)) << p.transpose();
    // The interpolant's gradient approximates the sphere's unit normal.
    if (p.norm() > 0.2)
    {
      EXPECT_LT((dense->getGradient(p) - p.normalized()).norm(), 0.1) << p.transpose();
    }
  }

  // A function-backed field takes central differences of its sampler.
  auto lazy = tesseract::geometry::createSignedDistanceField(
      SPHERE, Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5), Eigen::Vector3i::Constant(21));
  const Eigen::Vector3d p(0.2, -0.1, 0.15);
  EXPECT_TRUE(lazy->getGradient(p).isApprox(p.normalized(), 1e-6));
  Eigen::Vector3d combined;
  EXPECT_EQ(lazy->getDistanceAndGradient(p, combined), lazy->getDistance(p));
  EXPECT_EQ(combined, lazy->getGradient(p));

  // A sampler answering the wrong number of probes is rejected.
  const tesseract::geometry::BatchedSignedDistanceFunction one_value = [](const std::vector<Eigen::Vector3d>&) {
    return std::vector<double>{ 0.0 };
  };
  auto short_sampler = tesseract::geometry::createSignedDistanceField(
      one_value, Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5), Eigen::Vector3i::Constant(21));
  EXPECT_THROW(short_sampler->getGradient(p), std::runtime_error);  // NOLINT

  // Its probes stay inside the domain from a point on or near the boundary: the sampler may be defined only there.
  const Eigen::AlignedBox3d domain(Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5));
  const tesseract::geometry::BatchedSignedDistanceFunction inside_only =
      [&](const std::vector<Eigen::Vector3d>& points) {
        std::vector<double> distances;
        for (const Eigen::Vector3d& q : points)
        {
          if (!domain.contains(q))
            throw std::out_of_range("sampled outside the domain");
          distances.push_back(SPHERE(q));
        }
        return distances;
      };
  auto bounded = tesseract::geometry::createSignedDistanceField(
      inside_only, domain.min(), domain.max(), Eigen::Vector3i::Constant(21));
  for (const Eigen::Vector3d& q :
       { Eigen::Vector3d(0.5, 0.1, -0.2), Eigen::Vector3d(-0.5, -0.5, 0.5), Eigen::Vector3d(0.5 - 1e-9, 0.0, 0.0) })
  {
    Eigen::Vector3d gradient;
    EXPECT_NO_THROW(gradient = bounded->getGradient(q)) << q.transpose();  // NOLINT
    EXPECT_LT((gradient - q.normalized()).norm(), 1e-3) << q.transpose();
  }
}

TEST(TesseractGeometryUnit, SignedDistanceFieldDiscreteKeepsNoFunction)  // NOLINT
{
  // A field sampled up front answers from its grid: queries never call the function again.
  std::size_t calls = 0;
  const tesseract::geometry::SignedDistanceFunction counted = [&calls](const Eigen::Vector3d& p) {
    ++calls;
    return SPHERE(p);
  };
  auto field = tesseract::geometry::createDiscreteSignedDistanceField(
      counted, Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5), Eigen::Vector3i::Constant(11));
  EXPECT_EQ(calls, 11U * 11U * 11U);
  const Eigen::Vector3d p(0.2, -0.1, 0.15);
  Eigen::Vector3d gradient;
  field->getDistance(p);
  field->getGradient(p);
  field->getDistanceAndGradient(p, gradient);
  field->clone();
  EXPECT_EQ(calls, 11U * 11U * 11U);
}

TEST(TesseractGeometryUnit, SparseSignedDistanceFieldMatchesDense)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  const double voxel = 0.05;
  // Index -10..10 at 0.05 from the origin: the same lattice as the dense field below.
  const std::vector<std::uint8_t> grid = makeNanoVDBGrid(
      SPHERE, Eigen::Vector3d::Zero(), voxel, Eigen::Vector3i::Constant(-10), Eigen::Vector3i::Constant(10));
  const SignedDistanceField sparse(grid, 0.0);
  auto dense = makeGridField(SPHERE, 0.5, 21);

  EXPECT_TRUE(sparse.isSparse());
  EXPECT_FALSE(dense->isSparse());
  EXPECT_TRUE(dense->getNVDB().empty());
  EXPECT_FALSE(sparse.getNVDB().empty());
  EXPECT_EQ(sparse.getBandClamp(), 0.0);
  EXPECT_TRUE(sparse.getDomain().min().isApprox(Eigen::Vector3d::Constant(-0.5), 1e-12));
  EXPECT_TRUE(sparse.getDomain().max().isApprox(Eigen::Vector3d::Constant(0.5), 1e-12));
  EXPECT_EQ(sparse.getDimensions(), Eigen::Vector3i::Constant(21));
  EXPECT_ANY_THROW(sparse.getDistances());  // NOLINT

  // Same samples (stored as float), same trilinear rule: the fields agree to float precision.
  for (const Eigen::Vector3d& p : probePoints(0.49, voxel, 200))
  {
    EXPECT_NEAR(sparse.getDistance(p), dense->getDistance(p), 1e-6) << p.transpose();
    EXPECT_TRUE(sparse.getGradient(p).isApprox(dense->getGradient(p), 1e-5)) << p.transpose();
    EXPECT_TRUE(sparse.getGradient(p).isApprox(finiteDifferenceGradient(sparse, p, 1e-6), 1e-4)) << p.transpose();
  }

  // Outside the domain the grid has no answer: +infinity, zero gradient.
  EXPECT_EQ(sparse.getDistance(Eigen::Vector3d(0.6, 0, 0)), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(sparse.getGradient(Eigen::Vector3d(0.6, 0, 0)).isZero());

  // A copy is equal; a clone (new UUID) and an independently built field hold identical content.
  const SignedDistanceField copy = sparse;  // NOLINT(performance-unnecessary-copy-initialization)
  EXPECT_TRUE(copy == sparse);
  auto clone = std::static_pointer_cast<SignedDistanceField>(sparse.clone());
  EXPECT_TRUE(clone->isSparse());
  EXPECT_NE(clone->getUUID(), sparse.getUUID());
  EXPECT_TRUE(tesseract::geometry::isIdentical(*clone, sparse));
  EXPECT_EQ(clone->getDistance(Eigen::Vector3d(0.1, 0.2, 0.3)), sparse.getDistance(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(tesseract::geometry::isIdentical(SignedDistanceField(grid, 0.0), sparse));
  EXPECT_FALSE(
      tesseract::geometry::isIdentical(SignedDistanceField(grid, 0.0, Eigen::Vector3d::Constant(2.0)), sparse));
  EXPECT_FALSE(tesseract::geometry::isIdentical(SignedDistanceField(grid, 0.1), sparse));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*dense, sparse));
  EXPECT_FALSE(*dense == sparse);
}

TEST(TesseractGeometryUnit, SparseSignedDistanceFieldActivity)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  constexpr double inf = std::numeric_limits<double>::infinity();
  const double voxel = 0.05;
  // A narrow band: only voxels within two voxels of the surface are active. The inactive voxels
  // hold the background value 0, which would read as surface if it were blended in.
  const std::vector<std::uint8_t> band =
      makeNanoVDBGrid(SPHERE,
                      Eigen::Vector3d::Zero(),
                      voxel,
                      Eigen::Vector3i::Constant(-10),
                      Eigen::Vector3i::Constant(10),
                      [voxel](const Eigen::Vector3d& p) { return std::abs(SPHERE(p)) <= 2.0 * voxel; });
  const SignedDistanceField field(band, 0.0);

  // Near the surface every surrounding voxel is active: a real read.
  const Eigen::Vector3d on_band(0.31, 0.012, 0.007);
  EXPECT_NEAR(field.getDistance(on_band), SPHERE(on_band), 5e-3);
  // The centre is deep inside, where no voxel is active: no answer, not the background value 0.
  EXPECT_EQ(field.getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)), inf);
  EXPECT_TRUE(field.getGradient(Eigen::Vector3d(0.01, 0.02, 0.03)).isZero());
  // Where the eight voxels straddle the band edge, the read is refused too.
  const Eigen::Vector3d edge(0.3 - (2.0 * voxel) - (0.5 * voxel), 0.0, 0.0);
  EXPECT_EQ(field.getDistance(edge), inf);
  // A band clamp does not change what "no data" means.
  EXPECT_EQ(SignedDistanceField(band, 0.1).getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)), inf);

  // A saturated band: every voxel active, values clamped to +-0.1. A read at the clamp outside says
  // only "at least 0.1 away", so it is no answer; at the clamp inside it is a contact, as read.
  const std::vector<std::uint8_t> clamped_grid =
      makeNanoVDBGrid([](const Eigen::Vector3d& p) { return std::clamp(SPHERE(p), -0.1, 0.1); },
                      Eigen::Vector3d::Zero(),
                      voxel,
                      Eigen::Vector3i::Constant(-10),
                      Eigen::Vector3i::Constant(10));
  const SignedDistanceField clamped(clamped_grid, 0.1);
  const Eigen::Vector3d far_outside(0.46, 0.01, 0.02);
  EXPECT_EQ(clamped.getDistance(far_outside), inf);
  EXPECT_TRUE(clamped.getGradient(far_outside).isZero());
  EXPECT_NEAR(clamped.getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)), -0.1, 1e-6);
  EXPECT_NEAR(clamped.getDistance(on_band), SPHERE(on_band), 5e-3);
  // Without a clamp the same read is a distance.
  EXPECT_NEAR(SignedDistanceField(clamped_grid, 0.0).getDistance(far_outside), 0.1, 1e-6);

  // In a level set inactive voxels hold the background and are data. Here the interior beyond the
  // band is inactive with background -0.1, as a level set stores it.
  const auto active_outside = [voxel](const Eigen::Vector3d& p) { return SPHERE(p) >= -2.0 * voxel; };
  const std::vector<std::uint8_t> interior =
      makeNanoVDBGrid([](const Eigen::Vector3d& p) { return std::clamp(SPHERE(p), -0.1, 0.1); },
                      Eigen::Vector3d::Zero(),
                      voxel,
                      Eigen::Vector3i::Constant(-10),
                      Eigen::Vector3i::Constant(10),
                      active_outside,
                      -0.1F);
  const SignedDistanceField level_set(withGridClass(interior, 1), 0.1);
  EXPECT_NEAR(level_set.getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)), -0.1, 1e-6);
  EXPECT_NEAR(level_set.getDistance(edge), -0.1, 1e-6);
  // The same grid, not declared a level set: its inactive voxels are no data.
  const SignedDistanceField unknown(interior, 0.1);
  EXPECT_EQ(unknown.getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)), inf);
  EXPECT_EQ(unknown.getDistance(edge), inf);
  EXPECT_FALSE(tesseract::geometry::isIdentical(level_set, unknown));
}

TEST(TesseractGeometryUnit, SparseSignedDistanceFieldEncoding)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  const auto make_band = [](const std::string& name) {
    return withGridName(makeNanoVDBGrid([](const Eigen::Vector3d& p) { return std::clamp(SPHERE(p), -0.05, 0.05); },
                                        Eigen::Vector3d::Zero(),
                                        0.025,
                                        Eigen::Vector3i::Constant(-20),
                                        Eigen::Vector3i::Constant(20),
                                        [](const Eigen::Vector3d& p) { return std::abs(SPHERE(p)) <= 0.1; }),
                        name);
  };
  const SignedDistanceField field(make_band("part_a"), 0.05, Eigen::Vector3d::Constant(1e-3));

  // The grid is stored canonically: re-encoding a stored grid is a no-op.
  const SignedDistanceField again(field.getNVDB(), field.getBandClamp(), field.getScale());
  EXPECT_EQ(again.getNVDB(), field.getNVDB());
  EXPECT_TRUE(tesseract::geometry::isIdentical(again, field));

  // NanoVDB round trip: the grid, read back with its clamp as the same sparse field.
  const std::vector<std::uint8_t> nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(field);
  EXPECT_EQ(tesseract::geometry::getSignedDistanceFieldNVDBGridNames(nvdb), std::vector<std::string>{ "part_a" });
  auto read = tesseract::geometry::readSparseSignedDistanceFieldNVDB(nvdb, 0.05, field.getScale());
  ASSERT_TRUE(read->isSparse());
  EXPECT_EQ(read->getNVDB(), field.getNVDB());
  EXPECT_TRUE(tesseract::geometry::isIdentical(*read, field));
  EXPECT_EQ(tesseract::geometry::writeSignedDistanceFieldNVDB(*read), nvdb);
  for (const Eigen::Vector3d& p : probePoints(0.95, 0.025, 50))
    EXPECT_EQ(read->getDistance(p), field.getDistance(p)) << p.transpose();
  EXPECT_EQ(read->getDistance(Eigen::Vector3d(5, 5, 5)), std::numeric_limits<double>::infinity());
  // Selecting the only grid by name gives the same field.
  EXPECT_TRUE(tesseract::geometry::isIdentical(
      *tesseract::geometry::readSparseSignedDistanceFieldNVDB(nvdb, 0.05, field.getScale(), "part_a"), field));

  // A file holding several grids (concatenated NanoVDB files are one): each is read by name, and
  // nothing is assumed about how they relate.
  const SignedDistanceField other(make_band("part_b"), 0.05, Eigen::Vector3d::Constant(1e-3));
  std::vector<std::uint8_t> two = nvdb;
  const std::vector<std::uint8_t> other_nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(other);
  two.insert(two.end(), other_nvdb.begin(), other_nvdb.end());
  EXPECT_EQ(tesseract::geometry::getSignedDistanceFieldNVDBGridNames(two),
            (std::vector<std::string>{ "part_a", "part_b" }));
  EXPECT_TRUE(tesseract::geometry::isIdentical(
      *tesseract::geometry::readSparseSignedDistanceFieldNVDB(two, 0.05, field.getScale(), "part_b"), other));
  EXPECT_ANY_THROW(tesseract::geometry::readSparseSignedDistanceFieldNVDB(two, 0.05));                         // NOLINT
  EXPECT_ANY_THROW(tesseract::geometry::readSparseSignedDistanceFieldNVDB(two, 0.05, field.getScale(), "c"));  // NOLINT
  std::vector<std::uint8_t> twins = nvdb;
  twins.insert(twins.end(), nvdb.begin(), nvdb.end());
  EXPECT_ANY_THROW(  // NOLINT
      tesseract::geometry::readSparseSignedDistanceFieldNVDB(twins, 0.05, field.getScale(), "part_a"));
  EXPECT_ANY_THROW(tesseract::geometry::readSignedDistanceFieldNVDB(two));  // NOLINT

  // A single grid reads back dense, whatever it is named.
  auto dense = makeGridField(SPHERE, 0.5, 11);
  EXPECT_FALSE(
      tesseract::geometry::readSignedDistanceFieldNVDB(tesseract::geometry::writeSignedDistanceFieldNVDB(*dense))
          ->isSparse());
  auto field_dense = tesseract::geometry::readSignedDistanceFieldNVDB(nvdb);
  EXPECT_FALSE(field_dense->isSparse());
  // The file records the grid's active bounds, so the dense read spans them and no more.
  EXPECT_EQ(field_dense->getDimensions(), field.getDimensions());
  // A dense field's NanoVDB file is also a valid sparse field over the same samples.
  const SignedDistanceField from_dense(tesseract::geometry::writeSignedDistanceFieldNVDB(*dense), 0.0);
  EXPECT_NEAR(from_dense.getDistance(Eigen::Vector3d(0.11, 0.23, -0.07)),
              dense->getDistance(Eigen::Vector3d(0.11, 0.23, -0.07)),
              1e-6);

  // A sparse field is not one OpenVDB grid.
  EXPECT_ANY_THROW(tesseract::geometry::writeSignedDistanceFieldVDB(field));  // NOLINT
}

TEST(TesseractGeometryUnit, SparseSignedDistanceFieldLevels)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  constexpr double inf = std::numeric_limits<double>::infinity();
  // A fine narrow band (0.025 voxels, values clamped to +-0.05, active within 0.1 of the surface)
  // over a coarse grid of the whole box (0.05 voxels, every voxel active, never saturating).
  const std::vector<std::uint8_t> fine_grid =
      makeNanoVDBGrid([](const Eigen::Vector3d& p) { return std::clamp(SPHERE(p), -0.05, 0.05); },
                      Eigen::Vector3d::Zero(),
                      0.025,
                      Eigen::Vector3i::Constant(-20),
                      Eigen::Vector3i::Constant(20),
                      [](const Eigen::Vector3d& p) { return std::abs(SPHERE(p)) <= 0.1; });
  const std::vector<std::uint8_t> coarse_grid = makeNanoVDBGrid(
      SPHERE, Eigen::Vector3d::Zero(), 0.05, Eigen::Vector3i::Constant(-10), Eigen::Vector3i::Constant(10));
  const auto fine = std::make_shared<SignedDistanceField>(fine_grid, 0.05);
  const auto coarse = std::make_shared<SignedDistanceField>(coarse_grid, 0.0);
  const SignedDistanceField multi_level(std::vector<SignedDistanceField::ConstPtr>{ fine, coarse });
  EXPECT_TRUE(multi_level.isSparse());
  ASSERT_EQ(multi_level.getLevels().size(), 2U);
  EXPECT_TRUE(multi_level.getNVDB().empty());
  EXPECT_ANY_THROW(multi_level.getDistances());  // NOLINT
  EXPECT_TRUE(multi_level.getDomain().min().isApprox(Eigen::Vector3d::Constant(-0.5), 1e-12));
  EXPECT_EQ(multi_level.getDimensions(), Eigen::Vector3i::Constant(41));  // at the finest level's spacing

  // Within the fine band the fine level answers.
  const Eigen::Vector3d near(0.32, 0.011, 0.007);
  EXPECT_EQ(multi_level.getDistance(near), fine->getDistance(near));
  // Past the fine clamp, outside or inside, the fine level gives way to the coarse one: a saturated
  // read says only "at least this far", on either side.
  const Eigen::Vector3d outside(0.38, 0.012, 0.007);
  const Eigen::Vector3d inside(0.22, 0.012, 0.007);
  EXPECT_EQ(fine->getDistance(outside), inf);           // alone: saturated outside, no answer
  EXPECT_NEAR(fine->getDistance(inside), -0.05, 1e-6);  // alone: saturated inside, a contact
  for (const Eigen::Vector3d& p : { outside, inside })
  {
    EXPECT_EQ(multi_level.getDistance(p), coarse->getDistance(p)) << p.transpose();
    EXPECT_NEAR(multi_level.getDistance(p), SPHERE(p), 5e-3) << p.transpose();
    EXPECT_TRUE(multi_level.getGradient(p).isApprox(coarse->getGradient(p), 1e-12)) << p.transpose();
  }
  // Where the fine level has no data at all (the centre), the coarse level answers.
  EXPECT_EQ(multi_level.getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)),
            coarse->getDistance(Eigen::Vector3d(0.01, 0.02, 0.03)));
  // Where no level can answer, there is no answer.
  EXPECT_EQ(multi_level.getDistance(Eigen::Vector3d(5, 5, 5)), inf);
  // The multi-level field reads its levels with its own interpolation settings.
  SignedDistanceField cubic = multi_level;
  cubic.setInterpolation(tesseract::geometry::SignedDistanceFieldInterpolation::TRICUBIC);
  SignedDistanceField fine_cubic(fine_grid, 0.05);
  fine_cubic.setInterpolation(tesseract::geometry::SignedDistanceFieldInterpolation::TRICUBIC);
  EXPECT_EQ(cubic.getDistance(near), fine_cubic.getDistance(near));

  // Identity is the levels' grids and clamps, in order.
  const SignedDistanceField again(
      std::vector<SignedDistanceField::ConstPtr>{ std::make_shared<SignedDistanceField>(fine_grid, 0.05),
                                                  std::make_shared<SignedDistanceField>(coarse_grid, 0.0) });
  EXPECT_TRUE(tesseract::geometry::isIdentical(again, multi_level));
  const SignedDistanceField other_clamp(
      std::vector<SignedDistanceField::ConstPtr>{ std::make_shared<SignedDistanceField>(fine_grid, 0.04), coarse });
  EXPECT_FALSE(tesseract::geometry::isIdentical(other_clamp, multi_level));
  EXPECT_FALSE(tesseract::geometry::isIdentical(*fine, multi_level));
  auto clone = std::static_pointer_cast<SignedDistanceField>(multi_level.clone());
  EXPECT_NE(clone->getUUID(), multi_level.getUUID());
  EXPECT_TRUE(tesseract::geometry::isIdentical(*clone, multi_level));
  EXPECT_EQ(clone->getDistance(near), multi_level.getDistance(near));

  // Written as one file of the levels' grids in order, each read back by index.
  const std::vector<std::uint8_t> nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(multi_level);
  EXPECT_EQ(tesseract::geometry::getSignedDistanceFieldNVDBGridNames(nvdb).size(), 2U);
  EXPECT_TRUE(tesseract::geometry::isIdentical(
      *tesseract::geometry::readSparseSignedDistanceFieldNVDB(nvdb, 0.05, Eigen::Vector3d::Ones(), std::size_t{ 0 }),
      *fine));
  EXPECT_TRUE(tesseract::geometry::isIdentical(
      *tesseract::geometry::readSparseSignedDistanceFieldNVDB(nvdb, 0.0, Eigen::Vector3d::Ones(), std::size_t{ 1 }),
      *coarse));
  EXPECT_ANY_THROW(  // NOLINT
      tesseract::geometry::readSparseSignedDistanceFieldNVDB(nvdb, 0.0, Eigen::Vector3d::Ones(), std::size_t{ 2 }));

  // Refused: no levels, a null or non-sparse level, a multi-level field as a level, mismatched scales, and levels
  // not ordered finest first.
  using Levels = std::vector<SignedDistanceField::ConstPtr>;
  EXPECT_ANY_THROW(SignedDistanceField(Levels{}));                                        // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(Levels{ fine, nullptr }));                         // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(Levels{ fine, makeGridField(SPHERE, 0.5, 21) }));  // NOLINT
  EXPECT_ANY_THROW(
      SignedDistanceField(Levels{ std::make_shared<SignedDistanceField>(multi_level), coarse }));  // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(                                                            // NOLINT
      Levels{ fine, std::make_shared<SignedDistanceField>(coarse_grid, 0.0, Eigen::Vector3d::Constant(2.0)) }));
  EXPECT_ANY_THROW(SignedDistanceField(Levels{ coarse, fine }));  // NOLINT
  EXPECT_NO_THROW(SignedDistanceField(Levels{ coarse }));         // NOLINT
}

TEST(TesseractGeometryUnit, SparseSignedDistanceFieldRejectsInvalidGrids)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  const std::vector<std::uint8_t> grid = makeNanoVDBGrid(
      SPHERE, Eigen::Vector3d::Zero(), 0.1, Eigen::Vector3i::Constant(-3), Eigen::Vector3i::Constant(3));
  EXPECT_ANY_THROW(SignedDistanceField(std::vector<std::uint8_t>{}, 0.0));                // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(grid, -1.0));                                      // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(grid, std::numeric_limits<double>::quiet_NaN()));  // NOLINT
  EXPECT_ANY_THROW(SignedDistanceField(std::vector<std::uint8_t>(64, 7), 0.0));           // NOLINT
  std::vector<std::uint8_t> truncated(grid.begin(), grid.begin() + static_cast<std::ptrdiff_t>(grid.size() / 2));
  EXPECT_ANY_THROW(SignedDistanceField(truncated, 0.0));  // NOLINT
  EXPECT_NO_THROW(SignedDistanceField(grid, 0.0));        // NOLINT

  // A fog volume (class 2) and an unknown major version (offset 16 of the grid's header) are refused.
  EXPECT_ANY_THROW(SignedDistanceField(withGridClass(grid, 2), 0.0));  // NOLINT
  std::vector<std::uint8_t> future = grid;
  const std::uint32_t version = 33U << 21U;
  std::memcpy(future.data() + gridHeaderOffset(grid) + 16, &version, sizeof(version));
  EXPECT_ANY_THROW(SignedDistanceField(future, 0.0));  // NOLINT
}

TEST(TesseractGeometryUnit, SignedDistanceFieldInterpolation)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  using tesseract::geometry::SignedDistanceFieldInterpolation;
  const std::array<SignedDistanceFieldInterpolation, 2> methods{ SignedDistanceFieldInterpolation::TRILINEAR,
                                                                 SignedDistanceFieldInterpolation::TRICUBIC };

  // Every method reproduces a linear field, value and gradient, away from the domain faces.
  const tesseract::geometry::SignedDistanceFunction plane = [](const Eigen::Vector3d& p) {
    return (0.3 * p.x()) - (0.5 * p.y()) + (0.8 * p.z()) + 0.1;
  };
  auto linear = makeGridField(plane, 0.5, 21);
  for (auto method : methods)
  {
    linear->setInterpolation(method);
    EXPECT_EQ(linear->getInterpolation(), method);
    for (const Eigen::Vector3d& p : probePoints(0.35, 0.05, 30))
    {
      EXPECT_NEAR(linear->getDistance(p), plane(p), 1e-6) << static_cast<int>(method) << " " << p.transpose();
      EXPECT_TRUE(linear->getGradient(p).isApprox(Eigen::Vector3d(0.3, -0.5, 0.8), 1e-5)) << static_cast<int>(method);
    }
  }

  auto dense = makeGridField(SPHERE, 0.5, 21);
  SignedDistanceField sparse(
      makeNanoVDBGrid(
          SPHERE, Eigen::Vector3d::Zero(), 0.05, Eigen::Vector3i::Constant(-10), Eigen::Vector3i::Constant(10)),
      0.0);
  for (auto method : methods)
  {
    dense->setInterpolation(method);
    sparse.setInterpolation(method);
    for (const Eigen::Vector3d& p : probePoints(0.4, 0.05, 60))
    {
      // Sparse and dense agree under every method, and the gradient is the interpolant's own.
      EXPECT_NEAR(sparse.getDistance(p), dense->getDistance(p), 1e-6) << static_cast<int>(method);
      EXPECT_TRUE(sparse.getGradient(p).isApprox(dense->getGradient(p), 1e-5)) << static_cast<int>(method);
      EXPECT_TRUE(dense->getGradient(p).isApprox(finiteDifferenceGradient(*dense, p, 1e-6), 1e-4))
          << static_cast<int>(method) << " " << p.transpose();
      // All are close to the true sphere distance at this resolution, away from the centre (where
      // the true distance has a point the grid cannot hold).
      if (p.norm() > 0.15)
      {
        EXPECT_NEAR(dense->getDistance(p), SPHERE(p), 5e-3) << static_cast<int>(method);
      }
    }
  }

  // Tricubic passes through the samples.
  const Eigen::Vector3d node(0.15, -0.2, 0.25);  // a lattice point
  dense->setInterpolation(SignedDistanceFieldInterpolation::TRICUBIC);
  EXPECT_NEAR(dense->getDistance(node), static_cast<float>(SPHERE(node)), 1e-9);

  // Across a cell face (x = 0.2) the trilinear gradient jumps; the tricubic gradient does not.
  const Eigen::Vector3d below(0.2 - 1e-7, 0.13, 0.07);
  const Eigen::Vector3d above(0.2 + 1e-7, 0.13, 0.07);
  dense->setInterpolation(SignedDistanceFieldInterpolation::TRILINEAR);
  EXPECT_GT((dense->getGradient(below) - dense->getGradient(above)).norm(), 1e-3);
  dense->setInterpolation(SignedDistanceFieldInterpolation::TRICUBIC);
  EXPECT_LT((dense->getGradient(below) - dense->getGradient(above)).norm(), 1e-5);

  // Wider stencils need more active voxels: at the edge of a sparse grid's active region trilinear
  // still answers where tricubic cannot.
  const Eigen::Vector3d near_edge(0.47, 0.0, 0.0);  // between voxels 9 and 10, the last active one
  sparse.setInterpolation(SignedDistanceFieldInterpolation::TRILINEAR);
  EXPECT_TRUE(std::isfinite(sparse.getDistance(near_edge)));
  sparse.setInterpolation(SignedDistanceFieldInterpolation::TRICUBIC);
  EXPECT_EQ(sparse.getDistance(near_edge), std::numeric_limits<double>::infinity());

  // The method is part of the field's identity, and clones keep it.
  dense->setInterpolation(SignedDistanceFieldInterpolation::TRICUBIC);
  auto clone = std::static_pointer_cast<SignedDistanceField>(dense->clone());
  EXPECT_EQ(clone->getInterpolation(), SignedDistanceFieldInterpolation::TRICUBIC);
  EXPECT_TRUE(tesseract::geometry::isIdentical(*clone, *dense));
  clone->setInterpolation(SignedDistanceFieldInterpolation::TRILINEAR);
  EXPECT_FALSE(tesseract::geometry::isIdentical(*clone, *dense));
}

TEST(TesseractGeometryUnit, SignedDistanceFieldSplitInterpolation)  // NOLINT
{
  using tesseract::geometry::SignedDistanceField;
  using tesseract::geometry::SignedDistanceFieldInterpolation;
  constexpr auto LINEAR = SignedDistanceFieldInterpolation::TRILINEAR;
  constexpr auto CUBIC = SignedDistanceFieldInterpolation::TRICUBIC;

  auto linear = makeGridField(SPHERE, 0.5, 21);
  auto cubic = makeGridField(SPHERE, 0.5, 21);
  cubic->setInterpolation(CUBIC);
  auto split = makeGridField(SPHERE, 0.5, 21);
  split->setInterpolation(LINEAR);
  split->setGradientInterpolation(CUBIC);
  EXPECT_EQ(split->getInterpolation(), LINEAR);
  EXPECT_EQ(split->getGradientInterpolation(), CUBIC);

  const std::vector<std::uint8_t> grid = makeNanoVDBGrid(
      SPHERE, Eigen::Vector3d::Zero(), 0.05, Eigen::Vector3i::Constant(-10), Eigen::Vector3i::Constant(10));
  SignedDistanceField sparse_split(grid, 0.0);
  sparse_split.setInterpolation(LINEAR);
  sparse_split.setGradientInterpolation(CUBIC);

  for (const Eigen::Vector3d& p : probePoints(0.4, 0.05, 40))
  {
    // Distance from the distance method, direction from the direction method.
    EXPECT_DOUBLE_EQ(split->getDistance(p), linear->getDistance(p));
    EXPECT_TRUE(split->getGradient(p).isApprox(cubic->getGradient(p), 1e-12));
    // One call gives the same answers as two.
    Eigen::Vector3d g;
    EXPECT_DOUBLE_EQ(split->getDistanceAndGradient(p, g), split->getDistance(p));
    EXPECT_TRUE(g.isApprox(split->getGradient(p), 1e-12));
    // Sparse agrees with dense.
    EXPECT_NEAR(sparse_split.getDistanceAndGradient(p, g), split->getDistance(p), 1e-6);
    EXPECT_TRUE(g.isApprox(split->getGradient(p), 1e-5));
  }

  // Where the tricubic stencil reaches past the active voxels, the direction falls back to
  // trilinear rather than going blank.
  const Eigen::Vector3d near_edge(0.47, 0.01, 0.02);
  SignedDistanceField sparse_linear(grid, 0.0);
  Eigen::Vector3d g;
  EXPECT_DOUBLE_EQ(sparse_split.getDistanceAndGradient(near_edge, g), sparse_linear.getDistance(near_edge));
  EXPECT_TRUE(g.isApprox(sparse_linear.getGradient(near_edge), 1e-12));
  EXPECT_GT(g.norm(), 0.5);

  // setInterpolation sets both; the direction method is part of identity and survives clone.
  auto clone = std::static_pointer_cast<SignedDistanceField>(split->clone());
  EXPECT_EQ(clone->getGradientInterpolation(), CUBIC);
  EXPECT_TRUE(tesseract::geometry::isIdentical(*clone, *split));
  clone->setInterpolation(LINEAR);
  EXPECT_EQ(clone->getGradientInterpolation(), LINEAR);
  EXPECT_FALSE(tesseract::geometry::isIdentical(*clone, *split));
  EXPECT_TRUE(tesseract::geometry::isIdentical(*clone, *linear));
}

TEST(TesseractGeometryUnit, Octree)  // NOLINT
{
  using T = tesseract::geometry::Octree;

  tesseract::geometry::PointCloud pc;
  auto octree = tesseract::geometry::createOctree(pc, 0.01, false);
  auto geom = std::make_shared<T>(std::move(octree), tesseract::geometry::OctreeSubType::BOX, false);
  EXPECT_TRUE(geom->getOctree() != nullptr);
  EXPECT_TRUE(geom->getSubType() == tesseract::geometry::OctreeSubType::BOX);
  EXPECT_EQ(geom->getType(), tesseract::geometry::GeometryType::OCTREE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getOctree() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getSubType() == tesseract::geometry::OctreeSubType::BOX);
  EXPECT_EQ(geom_clone->getType(), tesseract::geometry::GeometryType::OCTREE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract::geometry::isIdentical(*geom, *geom_clone));
}

TEST(TesseractGeometryUnit, OctreeClonePreservesPrunedFlag)  // NOLINT
{
  using T = tesseract::geometry::Octree;

  tesseract::geometry::PointCloud pc;
  auto octree = tesseract::geometry::createOctree(pc, 0.01, false);
  auto geom = std::make_shared<T>(std::move(octree), tesseract::geometry::OctreeSubType::BOX, /*pruned=*/true);
  ASSERT_TRUE(geom->getPruned());

  auto geom_clone = std::static_pointer_cast<T>(geom->clone());
  EXPECT_TRUE(geom_clone->getPruned());
}

TEST(TesseractGeometryUnit, LoadMeshUnit)  // NOLINT
{
  using namespace tesseract::geometry;

  tesseract::common::GeneralResourceLocator locator;
  std::string mesh_file = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.stl")->getFilePath();
  std::vector<Mesh::Ptr> meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.dae")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 2);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);
  EXPECT_TRUE(meshes[1]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[1]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.dae")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 2 * 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 2 * 42);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/box_2m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 12);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 8);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/box_2m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 12);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 8);

  mesh_file = locator.locateResource("package://tesseract/support/meshes/box_2m.ply")->getFilePath();
  std::vector<ConvexMesh::Ptr> convex_meshes =
      createMeshFromPath<ConvexMesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, false);
  EXPECT_TRUE(convex_meshes.size() == 1);
  EXPECT_TRUE(convex_meshes[0]->getFaceCount() == 6);
  EXPECT_TRUE(convex_meshes[0]->getVertexCount() == 8);
}

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL

TEST(TesseractGeometryUnit, LoadMeshWithMaterialGltf2Unit)  // NOLINT
{
  using namespace tesseract::geometry;

  tesseract::common::GeneralResourceLocator locator;
  std::string mesh_file =
      locator.locateResource("package://tesseract/support/meshes/tesseract_material_mesh.glb")->getFilePath();
  std::vector<Mesh::Ptr> meshes =
      createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true, true, true, true);
  ASSERT_TRUE(meshes.size() == 4);

  auto& mesh0 = meshes[0];
  auto& mesh1 = meshes[1];
  auto& mesh2 = meshes[2];
  auto& mesh3 = meshes[3];

  EXPECT_EQ(mesh0->getFaceCount(), 34);
  EXPECT_EQ(mesh0->getVertexCount(), 68);
  EXPECT_EQ(mesh1->getFaceCount(), 15);
  EXPECT_EQ(mesh1->getVertexCount(), 17);
  EXPECT_EQ(mesh2->getFaceCount(), 15);
  EXPECT_EQ(mesh2->getVertexCount(), 17);
  EXPECT_EQ(mesh3->getFaceCount(), 2);
  EXPECT_EQ(mesh3->getVertexCount(), 4);

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
  EXPECT_EQ(texture->getType(), MeshTextureType::BASE_COLOR);
  EXPECT_EQ(texture->getUVChannel(), 0);
}

TEST(TesseractGeometryUnit, LoadMeshWithPbrMaterialGltf2Unit)  // NOLINT
{
  using namespace tesseract::geometry;

  tesseract::common::GeneralResourceLocator locator;
  auto resource = locator.locateResource("package://tesseract/support/meshes/tesseract_pbr_mesh.glb");
  ASSERT_TRUE(resource != nullptr);
  std::vector<Mesh::Ptr> meshes =
      createMeshFromResource<Mesh>(resource, Eigen::Vector3d(1, 1, 1), true, true, true, true, true);
  ASSERT_EQ(meshes.size(), 2);

  // The mesh with textures is "pbr_full", the other "pbr_mask"
  const bool first_is_full = meshes[0]->getTextures() != nullptr;
  const Mesh::Ptr& full = first_is_full ? meshes[0] : meshes[1];
  const Mesh::Ptr& mask = first_is_full ? meshes[1] : meshes[0];

  // pbr_full: every texture slot, blended, double sided, with the extension and texture parameters
  auto material = full->getMaterial();
  ASSERT_TRUE(material != nullptr);
  EXPECT_TRUE(material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.9, 0.8, 0.7, 0.6), 1e-6));
  EXPECT_NEAR(material->getMetallicFactor(), 0.75, 1e-6);
  EXPECT_NEAR(material->getRoughnessFactor(), 0.25, 1e-6);
  EXPECT_TRUE(material->getEmissiveFactor().head<3>().isApprox(Eigen::Vector3d(1.0, 0.5, 0.25), 1e-6));
  EXPECT_EQ(material->getAlphaMode(), MeshMaterialAlphaMode::BLEND);
  EXPECT_TRUE(material->getDoubleSided());
  // assimp imports KHR_materials_emissive_strength from 5.3. Older versions leave the default.
  const bool imports_emissive_strength =
      aiGetVersionMajor() > 5 || (aiGetVersionMajor() == 5 && aiGetVersionMinor() >= 3);
  EXPECT_NEAR(material->getEmissiveStrength(), imports_emissive_strength ? 2.5 : 1.0, 1e-6);
  EXPECT_NEAR(material->getNormalScale(), 0.5, 1e-6);
  EXPECT_NEAR(material->getOcclusionStrength(), 0.8, 1e-6);

  const auto& textures = full->getTextures();
  ASSERT_TRUE(textures != nullptr);
  ASSERT_EQ(textures->size(), 5);
  std::map<MeshTextureType, MeshTexture::Ptr> by_type;
  for (const auto& texture : *textures)
    by_type[texture->getType()] = texture;
  ASSERT_EQ(by_type.size(), 5);

  for (const auto& [type, texture] : by_type)
  {
    ASSERT_TRUE(texture->getTextureImage() != nullptr);
    ASSERT_TRUE(texture->getUVs() != nullptr);
    EXPECT_EQ(texture->getUVs()->size(), full->getVertexCount());
    const std::string url = texture->getTextureImage()->getUrl();
    const std::string ext = (type == MeshTextureType::EMISSIVE) ? ".jpg" : ".png";
    EXPECT_EQ(url.substr(url.size() - ext.size()), ext) << url;
  }

  const auto& base = by_type.at(MeshTextureType::BASE_COLOR);
  const auto& occlusion = by_type.at(MeshTextureType::OCCLUSION);
  EXPECT_EQ(base->getUVChannel(), 0);
  EXPECT_EQ(by_type.at(MeshTextureType::METALLIC_ROUGHNESS)->getUVChannel(), 0);
  EXPECT_EQ(by_type.at(MeshTextureType::NORMAL)->getUVChannel(), 0);
  EXPECT_EQ(by_type.at(MeshTextureType::EMISSIVE)->getUVChannel(), 0);
  EXPECT_EQ(occlusion->getUVChannel(), 1);

  // Textures on one UV channel share the UV vector. TEXCOORD_1 spans half of TEXCOORD_0.
  EXPECT_EQ(base->getUVs(), by_type.at(MeshTextureType::NORMAL)->getUVs());
  EXPECT_NE(base->getUVs(), occlusion->getUVs());
  double max_u0 = 0;
  double max_u1 = 0;
  for (const auto& uv : *base->getUVs())
    max_u0 = std::max(max_u0, uv.x());
  for (const auto& uv : *occlusion->getUVs())
    max_u1 = std::max(max_u1, uv.x());
  EXPECT_NEAR(max_u0, 1.0, 1e-6);
  EXPECT_NEAR(max_u1, 0.5, 1e-6);

  // The base color texture has its own sampler, the others the glTF defaults
  EXPECT_EQ(base->getWrapU(), MeshTextureWrapMode::CLAMP_TO_EDGE);
  EXPECT_EQ(base->getWrapV(), MeshTextureWrapMode::MIRRORED_REPEAT);
  EXPECT_EQ(base->getMagFilter(), MeshTextureFilter::NEAREST);
  EXPECT_EQ(base->getMinFilter(), MeshTextureFilter::NEAREST);
  EXPECT_EQ(occlusion->getWrapU(), MeshTextureWrapMode::REPEAT);
  EXPECT_EQ(occlusion->getWrapV(), MeshTextureWrapMode::REPEAT);

  // pbr_mask: factors only, alpha masked
  auto mask_material = mask->getMaterial();
  ASSERT_TRUE(mask_material != nullptr);
  EXPECT_TRUE(mask_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.2, 0.4, 0.6, 1.0), 1e-6));
  EXPECT_NEAR(mask_material->getMetallicFactor(), 0.1, 1e-6);
  EXPECT_NEAR(mask_material->getRoughnessFactor(), 0.9, 1e-6);
  EXPECT_EQ(mask_material->getAlphaMode(), MeshMaterialAlphaMode::MASK);
  EXPECT_NEAR(mask_material->getAlphaCutoff(), 0.3, 1e-6);
  EXPECT_FALSE(mask_material->getDoubleSided());
  EXPECT_NEAR(mask_material->getEmissiveStrength(), 1.0, 1e-6);
  EXPECT_TRUE(mask->getTextures() == nullptr);

  // Loading without materials drops them (and lets assimp merge the two quads)
  std::vector<Mesh::Ptr> plain = createMeshFromResource<Mesh>(resource, Eigen::Vector3d(1, 1, 1), true, true);
  ASSERT_FALSE(plain.empty());
  for (const auto& mesh : plain)
  {
    EXPECT_TRUE(mesh->getMaterial() == nullptr);
    EXPECT_TRUE(mesh->getTextures() == nullptr);
  }
}
#endif

TEST(TesseractGeometryUnit, LoadMeshNormalsAreUnitAndIgnoreTranslation)  // NOLINT
{
  using namespace tesseract::geometry;
  tesseract::common::GeneralResourceLocator locator;

  // The quads of tesseract_pbr_mesh.glb face +Z. A non-uniform scale must not tilt or stretch their normals.
  auto pbr = locator.locateResource("package://tesseract/support/meshes/tesseract_pbr_mesh.glb");
  for (const auto& mesh : createMeshFromResource<Mesh>(pbr, Eigen::Vector3d(2, 3, 4), true, true, true))
  {
    ASSERT_TRUE(mesh->getNormals() != nullptr);
    for (const auto& n : *mesh->getNormals())
      EXPECT_TRUE(n.isApprox(Eigen::Vector3d(0, 0, 1), 1e-6)) << n.transpose();
  }

  // tesseract_material_mesh.glb has translated nodes
  auto material_mesh = locator.locateResource("package://tesseract/support/meshes/tesseract_material_mesh.glb");
  for (const auto& mesh : createMeshFromResource<Mesh>(material_mesh, Eigen::Vector3d(0.5, 2, 1), true, true, true))
  {
    ASSERT_TRUE(mesh->getNormals() != nullptr);
    for (const auto& n : *mesh->getNormals())
      EXPECT_NEAR(n.norm(), 1.0, 1e-6);
  }
}

TEST(TesseractGeometryUnit, MeshMaterialUnit)  // NOLINT
{
  using namespace tesseract::geometry;

  MeshMaterial material;
  EXPECT_TRUE(material.getBaseColorFactor().isApprox(Eigen::Vector4d::Zero()));
  EXPECT_DOUBLE_EQ(material.getMetallicFactor(), 0.0);
  EXPECT_DOUBLE_EQ(material.getRoughnessFactor(), 0.5);
  EXPECT_EQ(material.getAlphaMode(), MeshMaterialAlphaMode::OPAQUE);
  EXPECT_DOUBLE_EQ(material.getAlphaCutoff(), 0.5);
  EXPECT_FALSE(material.getDoubleSided());
  EXPECT_DOUBLE_EQ(material.getEmissiveStrength(), 1.0);
  EXPECT_DOUBLE_EQ(material.getNormalScale(), 1.0);
  EXPECT_DOUBLE_EQ(material.getOcclusionStrength(), 1.0);

  MeshMaterial other(Eigen::Vector4d(0, 0, 0, 0), 0, 0.5, Eigen::Vector4d(0, 0, 0, 0));
  EXPECT_TRUE(material == other);

  const auto expect_differs = [&material](const std::function<void(MeshMaterial&)>& change) {
    MeshMaterial changed = material;
    change(changed);
    EXPECT_TRUE(changed != material);
  };
  expect_differs([](MeshMaterial& m) { m.setBaseColorFactor(Eigen::Vector4d(1, 0, 0, 1)); });
  expect_differs([](MeshMaterial& m) { m.setMetallicFactor(0.3); });
  expect_differs([](MeshMaterial& m) { m.setRoughnessFactor(0.3); });
  expect_differs([](MeshMaterial& m) { m.setEmissiveFactor(Eigen::Vector4d(1, 0, 0, 1)); });
  expect_differs([](MeshMaterial& m) { m.setAlphaMode(MeshMaterialAlphaMode::MASK); });
  expect_differs([](MeshMaterial& m) { m.setAlphaCutoff(0.2); });
  expect_differs([](MeshMaterial& m) { m.setDoubleSided(true); });
  expect_differs([](MeshMaterial& m) { m.setEmissiveStrength(3.0); });
  expect_differs([](MeshMaterial& m) { m.setNormalScale(2.0); });
  expect_differs([](MeshMaterial& m) { m.setOcclusionStrength(0.1); });
}

TEST(TesseractGeometryUnit, MeshTextureUnit)  // NOLINT
{
  using namespace tesseract::geometry;

  auto uvs = std::make_shared<tesseract::common::VectorVector2d>();
  uvs->emplace_back(0, 0);
  uvs->emplace_back(1, 0.5);
  const std::vector<std::uint8_t> bytes = { 1, 2, 3 };
  auto image = std::make_shared<tesseract::common::BytesResource>("data://image.png", bytes);

  MeshTexture texture(image, uvs);
  EXPECT_EQ(texture.getType(), MeshTextureType::BASE_COLOR);
  EXPECT_EQ(texture.getUVChannel(), 0);
  EXPECT_EQ(texture.getWrapU(), MeshTextureWrapMode::REPEAT);
  EXPECT_EQ(texture.getWrapV(), MeshTextureWrapMode::REPEAT);
  EXPECT_EQ(texture.getMagFilter(), MeshTextureFilter::DEFAULT);
  EXPECT_EQ(texture.getMinFilter(), MeshTextureFilter::DEFAULT);
  EXPECT_EQ(texture.getUVs(), uvs);
  EXPECT_EQ(texture.getTextureImage(), image);

  // Equal settings, UV values and image contents compare equal, even in different objects
  auto uvs_copy = std::make_shared<tesseract::common::VectorVector2d>(*uvs);
  auto image_copy = std::make_shared<tesseract::common::BytesResource>("data://other.png", bytes);
  EXPECT_TRUE(texture == MeshTexture(image_copy, uvs_copy));

  EXPECT_TRUE(texture != MeshTexture(image, uvs, MeshTextureType::EMISSIVE));
  EXPECT_TRUE(texture != MeshTexture(image, uvs, MeshTextureType::BASE_COLOR, 1));
  EXPECT_TRUE(texture != MeshTexture(nullptr, uvs));
  EXPECT_TRUE(texture != MeshTexture(image, nullptr));
  EXPECT_TRUE(texture != MeshTexture(std::make_shared<tesseract::common::BytesResource>(
                                         "data://image.png", std::vector<std::uint8_t>{ 1, 2, 4 }),
                                     uvs));
  auto uvs_moved = std::make_shared<tesseract::common::VectorVector2d>(*uvs);
  uvs_moved->back().y() = 0.25;
  EXPECT_TRUE(texture != MeshTexture(image, uvs_moved));

  MeshTexture wrapped(image, uvs);
  wrapped.setWrap(MeshTextureWrapMode::CLAMP_TO_EDGE, MeshTextureWrapMode::MIRRORED_REPEAT);
  EXPECT_EQ(wrapped.getWrapU(), MeshTextureWrapMode::CLAMP_TO_EDGE);
  EXPECT_EQ(wrapped.getWrapV(), MeshTextureWrapMode::MIRRORED_REPEAT);
  EXPECT_TRUE(texture != wrapped);

  MeshTexture filtered(image, uvs);
  filtered.setFilters(MeshTextureFilter::LINEAR, MeshTextureFilter::LINEAR_MIPMAP_LINEAR);
  EXPECT_EQ(filtered.getMagFilter(), MeshTextureFilter::LINEAR);
  EXPECT_EQ(filtered.getMinFilter(), MeshTextureFilter::LINEAR_MIPMAP_LINEAR);
  EXPECT_TRUE(texture != filtered);
}

// Helper to create a trivial single-triangle mesh
static std::shared_ptr<tesseract::geometry::Mesh> makeSimpleTriangleMesh()
{
  auto verts = std::make_shared<tesseract::common::VectorVector3d>();
  verts->emplace_back(0, 0, 0);
  verts->emplace_back(1, 0, 0);
  verts->emplace_back(0, 1, 0);

  std::vector<int> idx = { 3, 0, 1, 2 };
  auto faces = std::make_shared<Eigen::VectorXi>(Eigen::Map<Eigen::VectorXi>(idx.data(), static_cast<int>(idx.size())));
  return std::make_shared<tesseract::geometry::Mesh>(verts, faces, nullptr, Eigen::Vector3d(1, 1, 1), verts);
}

// Generic translation test for primitives
static void checkTranslationInvariant(const std::shared_ptr<tesseract::geometry::Geometry>& geom)
{
  // Identity
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  tesseract::common::VectorVector3d out_id = tesseract::geometry::extractVertices(*geom, origin);
  ASSERT_GT(out_id.size(), 0U);

  // Translation
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translate(Eigen::Vector3d{ 1.5, -2.0, 0.75 });
  tf.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  tesseract::common::VectorVector3d out_tr = tesseract::geometry::extractVertices(*geom, tf);
  ASSERT_EQ(out_tr.size(), out_id.size());
  for (size_t i = 0; i < out_id.size(); ++i)
  {
    EXPECT_TRUE(out_tr[i].isApprox(tf * out_id[i], 1e-6));
  }
}

TEST(TesseractGeometryUnit, ExtractVerticesBox)
{
  checkTranslationInvariant(std::make_shared<tesseract::geometry::Box>(1.0, 2.0, 3.0));
}
TEST(TesseractGeometryUnit, ExtractVerticesSphere)
{
  checkTranslationInvariant(std::make_shared<tesseract::geometry::Sphere>(1.0));
}
TEST(TesseractGeometryUnit, ExtractVerticesCylinder)
{
  checkTranslationInvariant(std::make_shared<tesseract::geometry::Cylinder>(0.5, 1.0));
}
TEST(TesseractGeometryUnit, ExtractVerticesCone)
{
  checkTranslationInvariant(std::make_shared<tesseract::geometry::Cone>(0.5, 1.0));
}
TEST(TesseractGeometryUnit, ExtractVerticesCapsule)
{
  checkTranslationInvariant(std::make_shared<tesseract::geometry::Capsule>(0.5, 1.0));
}
TEST(TesseractGeometryUnit, ExtractVerticesPlane)
{
  EXPECT_ANY_THROW(checkTranslationInvariant(std::make_shared<tesseract::geometry::Plane>(0, 0, 1, 0)));  // NOLINT
}

TEST(TesseractGeometryUnit, ExtractVerticesMesh)
{
  std::shared_ptr<tesseract::geometry::Mesh> geom = makeSimpleTriangleMesh();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translate(Eigen::Vector3d{ 1.0, 2.0, 3.0 });
  tf.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

  tesseract::common::VectorVector3d out = tesseract::geometry::extractVertices(*geom, tf);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_TRUE(out[0].isApprox(tf * Eigen::Vector3d(0, 0, 0), 1e-6));
  EXPECT_TRUE(out[1].isApprox(tf * Eigen::Vector3d(1, 0, 0), 1e-6));
  EXPECT_TRUE(out[2].isApprox(tf * Eigen::Vector3d(0, 1, 0), 1e-6));
}

TEST(TesseractGeometryUnit, ExtractVerticesConvexMesh)
{
  auto simple = makeSimpleTriangleMesh();
  auto geom = std::make_shared<tesseract::geometry::ConvexMesh>(
      simple->getVertices(), simple->getFaces(), 1, nullptr, Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  tesseract::common::VectorVector3d id = tesseract::geometry::extractVertices(*geom, origin);
  origin = Eigen::Translation3d(0.2, 0.3, 0.4);
  tesseract::common::VectorVector3d tr = tesseract::geometry::extractVertices(*geom, origin);
  ASSERT_EQ(id.size(), tr.size());
  for (size_t i = 0; i < id.size(); ++i)
  {
    EXPECT_TRUE(tr[i].isApprox(id[i] + Eigen::Vector3d(0.2, 0.3, 0.4), 1e-6));
  }
}

TEST(TesseractGeometryUnit, ExtractVerticesPolygonMesh)
{
  auto simple = makeSimpleTriangleMesh();
  auto geom = std::make_shared<tesseract::geometry::PolygonMesh>(
      simple->getVertices(), simple->getFaces(), 1, nullptr, Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  tesseract::common::VectorVector3d id = tesseract::geometry::extractVertices(*geom, origin);
  origin = Eigen::Translation3d(0.2, 0.3, 0.4);
  tesseract::common::VectorVector3d tr = tesseract::geometry::extractVertices(*geom, origin);
  ASSERT_EQ(id.size(), tr.size());
  for (size_t i = 0; i < id.size(); ++i)
  {
    EXPECT_TRUE(tr[i].isApprox(id[i] + Eigen::Vector3d(0.2, 0.3, 0.4), 1e-6));
  }
}

TEST(TesseractGeometryUnit, ExtractVerticesCompoundMesh)
{
  std::vector<std::shared_ptr<tesseract::geometry::Mesh>> c{ makeSimpleTriangleMesh(), makeSimpleTriangleMesh() };
  auto geom = std::make_shared<tesseract::geometry::CompoundMesh>(c);
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  tesseract::common::VectorVector3d id = tesseract::geometry::extractVertices(*geom, origin);
  origin = Eigen::Translation3d(-1, 1, 0);
  tesseract::common::VectorVector3d tr = tesseract::geometry::extractVertices(*geom, origin);
  ASSERT_EQ(id.size(), tr.size());
  for (size_t i = 0; i < id.size(); ++i)
  {
    EXPECT_TRUE(tr[i].isApprox(id[i] + Eigen::Vector3d(-1, 1, 0), 1e-6));
  }
}

struct DummyGeometry : public tesseract::geometry::Geometry
{
  DummyGeometry() : Geometry(tesseract::geometry::GeometryType::OCTREE) {}
  std::shared_ptr<Geometry> clone() const override { return nullptr; }
};

TEST(TesseractGeometryUnit, ExtractVerticesUnsupported)
{
  auto geom = std::make_shared<DummyGeometry>();
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  EXPECT_ANY_THROW(tesseract::geometry::extractVertices(*geom, origin));  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
