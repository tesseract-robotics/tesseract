#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <system_error>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/common/ply_io.h>
#include <tesseract/geometry/impl/convex_mesh.h>
#include <tesseract/geometry/mesh_parser.h>

namespace
{
using Vertices = tesseract::common::VectorVector3d;
using Faces = std::set<std::set<int>>;

Vertices cube()
{
  Vertices vertices;
  for (double x : { -0.5, 0.5 })
    for (double y : { -0.5, 0.5 })
      for (double z : { -0.5, 0.5 })
        vertices.emplace_back(x, y, z);
  return vertices;
}

const Faces CUBE_FACES{
  { 0, 1, 2, 3 }, { 4, 5, 6, 7 }, { 0, 1, 4, 5 }, { 2, 3, 6, 7 }, { 0, 2, 4, 6 }, { 1, 3, 5, 7 }
};
const Vertices TETRAHEDRON{ { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
const Faces TETRAHEDRON_FACES{ { 0, 1, 2 }, { 0, 1, 3 }, { 0, 2, 3 }, { 1, 2, 3 } };
const Vertices PRISM{ { 0, 0, -1 }, { 1, 0, -1 }, { 0, 1, -1 }, { 0, 0, 1 }, { 1, 0, 1 }, { 0, 1, 1 } };
const Faces PRISM_FACES{ { 0, 1, 2 }, { 3, 4, 5 }, { 0, 1, 3, 4 }, { 0, 2, 3, 5 }, { 1, 2, 4, 5 } };

void checkHull(const Vertices& vertices,
               const Eigen::VectorXi& faces,
               int face_count,
               const Vertices& input,
               const Faces& expected)
{
  ASSERT_GT(face_count, 0);
  ASSERT_EQ(static_cast<std::size_t>(face_count), expected.size());
  ASSERT_EQ(vertices.size(), input.size());

  std::vector<int> input_indices;
  std::set<int> used_vertices;
  for (const auto& vertex : vertices)
  {
    std::size_t match = input.size();
    for (std::size_t i = 0; i < input.size(); ++i)
      if ((vertex - input[i]).norm() < 1e-6)
      {
        match = i;
        break;
      }
    ASSERT_LT(match, input.size());
    const int index = static_cast<int>(match);
    ASSERT_TRUE(used_vertices.insert(index).second);
    input_indices.push_back(index);
  }

  Faces actual;
  Eigen::Index offset = 0;
  for (int i = 0; i < face_count; ++i)
  {
    ASSERT_LT(offset, faces.size());
    const int count = faces[offset++];
    ASSERT_GE(count, 3);
    ASSERT_LE(offset + count, faces.size());
    std::set<int> polygon;
    for (int j = 0; j < count; ++j)
    {
      const int index = faces[offset++];
      ASSERT_GE(index, 0);
      ASSERT_LT(static_cast<std::size_t>(index), vertices.size());
      ASSERT_TRUE(polygon.insert(input_indices[static_cast<std::size_t>(index)]).second);
    }
    ASSERT_TRUE(actual.insert(polygon).second);
  }
  EXPECT_EQ(offset, faces.size());
  EXPECT_EQ(actual, expected);
}

struct PlyFile
{
  PlyFile() = default;
  PlyFile(const PlyFile&) = delete;
  PlyFile& operator=(const PlyFile&) = delete;
  PlyFile(PlyFile&&) = delete;
  PlyFile& operator=(PlyFile&&) = delete;

  std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("tesseract-convex-hull-" + boost::uuids::to_string(boost::uuids::random_generator()()) + ".ply");
  ~PlyFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};

/** @brief The 12 triangles of cube() as a face list */
Eigen::VectorXi cubeTriangles()
{
  Eigen::VectorXi triangles(48);
  triangles << 3, 0, 2, 3, 3, 0, 3, 1, 3, 4, 5, 7, 3, 4, 7, 6, 3, 0, 1, 5, 3, 0, 5, 4, 3, 2, 6, 7, 3, 2, 7, 3, 3, 0, 4,
      6, 3, 0, 6, 2, 3, 1, 3, 7, 3, 1, 7, 5;
  return triangles;
}

/** @brief Points on the unit sphere (Fibonacci spiral), so every point is a vertex of the convex hull */
std::shared_ptr<const Vertices> spherePoints(int count)
{
  auto vertices = std::make_shared<Vertices>();
  vertices->reserve(static_cast<std::size_t>(count));
  const double golden_angle = 2.39996322972865332;  // pi * (3 - sqrt(5))
  for (int i = 0; i < count; ++i)
  {
    const double z = 1.0 - (2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(count));
    const double r = std::sqrt(1.0 - (z * z));
    const double angle = golden_angle * static_cast<double>(i);
    vertices->emplace_back(r * std::cos(angle), r * std::sin(angle), z);
  }
  return vertices;
}

/** @brief Time in seconds to build the Bullet shape of a convex mesh with the given vertices */
double convexMeshBuildSeconds(const std::shared_ptr<const Vertices>& vertices)
{
  // The Bullet shape is built from the vertices only, so one face is enough. A new geometry has a new UUID, so
  // the shape cache does not return a shape built earlier.
  auto faces = std::make_shared<Eigen::VectorXi>(4);
  *faces << 3, 0, 1, 2;
  const auto geom = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 1);

  const auto start = std::chrono::steady_clock::now();
  const auto shape = tesseract::collision::bullet_internal::createShapePrimitive(geom);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  EXPECT_NE(shape, nullptr);
  return elapsed;
}
}  // namespace

TEST(TesseractConvexHullUnit, CubeFaces)  // NOLINT
{
  const Vertices input = cube();
  Vertices vertices;
  Eigen::VectorXi faces;
  const int count = tesseract::collision::createConvexHull(vertices, faces, input);
  checkHull(vertices, faces, count, input, CUBE_FACES);
}

TEST(TesseractConvexHullUnit, TriangleFaces)  // NOLINT
{
  Vertices vertices;
  Eigen::VectorXi faces;
  const int count = tesseract::collision::createConvexHull(vertices, faces, TETRAHEDRON);
  checkHull(vertices, faces, count, TETRAHEDRON, TETRAHEDRON_FACES);
}

TEST(TesseractConvexHullUnit, RepeatedGeometry)  // NOLINT
{
  Vertices vertices;
  Eigen::VectorXi faces;
  const std::vector<Vertices> inputs{ cube(), PRISM, TETRAHEDRON, cube() };
  const std::vector<Faces> expected{ CUBE_FACES, PRISM_FACES, TETRAHEDRON_FACES, CUBE_FACES };
  for (std::size_t i = 0; i < inputs.size(); ++i)
  {
    SCOPED_TRACE(i);
    const int count = tesseract::collision::createConvexHull(vertices, faces, inputs[i]);
    ASSERT_NO_FATAL_FAILURE(checkHull(vertices, faces, count, inputs[i], expected[i]));
  }
}

TEST(TesseractConvexHullUnit, MeshPlyRoundTrip)  // NOLINT
{
  const auto input = std::make_shared<Vertices>(cube());
  auto triangles = std::make_shared<Eigen::VectorXi>(48);
  *triangles << 3, 0, 2, 3, 3, 0, 3, 1, 3, 4, 5, 7, 3, 4, 7, 6, 3, 0, 1, 5, 3, 0, 5, 4, 3, 2, 6, 7, 3, 2, 7, 3, 3, 0, 4,
      6, 3, 0, 6, 2, 3, 1, 3, 7, 3, 1, 7, 5;
  const tesseract::geometry::Mesh mesh(input, triangles, 12);
  const auto convex = tesseract::collision::makeConvexMesh(mesh);
  ASSERT_NE(convex, nullptr);
  ASSERT_NO_FATAL_FAILURE(
      checkHull(*convex->getVertices(), *convex->getFaces(), convex->getFaceCount(), *input, CUBE_FACES));

  const PlyFile file;
  ASSERT_TRUE(tesseract::common::writeSimplePlyFile(
      file.path.string(), *convex->getVertices(), *convex->getFaces(), convex->getFaceCount()));

  Vertices loaded_vertices;
  Eigen::VectorXi loaded_faces;
  const int loaded_count =
      tesseract::common::loadSimplePlyFile(file.path.string(), loaded_vertices, loaded_faces, false);
  ASSERT_NO_FATAL_FAILURE(checkHull(loaded_vertices, loaded_faces, loaded_count, *input, CUBE_FACES));

  const auto loaded = tesseract::geometry::createMeshFromPath<tesseract::geometry::ConvexMesh>(
      file.path.string(), Eigen::Vector3d::Ones(), false, false);
  ASSERT_EQ(loaded.size(), 1U);
  ASSERT_NE(loaded.front(), nullptr);
  checkHull(
      *loaded.front()->getVertices(), *loaded.front()->getFaces(), loaded.front()->getFaceCount(), *input, CUBE_FACES);
}

/**
 * A convex mesh becomes a btConvexHullShape, which caches its local AABB grown by the margin the shape has when
 * recalcLocalAabb() runs. setMargin() does not update that cache. The points must therefore be added with
 * BULLET_MARGIN already set, or the AABB keeps Bullet's default CONVEX_DISTANCE_MARGIN (0.04) on every side.
 */
TEST(TesseractConvexHullUnit, BulletConvexMeshAabbUsesBulletMargin)  // NOLINT
{
  using tesseract::collision::bullet_internal::BULLET_MARGIN;

  const auto geom = std::make_shared<tesseract::geometry::ConvexMesh>(
      std::make_shared<const Vertices>(cube()), std::make_shared<const Eigen::VectorXi>(cubeTriangles()), 12);

  const auto shape = tesseract::collision::bullet_internal::createShapePrimitive(geom);
  ASSERT_NE(shape, nullptr);
  ASSERT_NE(shape->top_level, nullptr);
  ASSERT_EQ(shape->top_level->getShapeType(), CONVEX_HULL_SHAPE_PROXYTYPE);
  const auto* hull = static_cast<const btConvexHullShape*>(shape->top_level.get());
  EXPECT_EQ(hull->getNumPoints(), 8);
  EXPECT_NEAR(hull->getMargin(), BULLET_MARGIN, 1e-12);

  btTransform identity;
  identity.setIdentity();
  btVector3 aabb_min;
  btVector3 aabb_max;
  hull->getAabb(identity, aabb_min, aabb_max);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(aabb_min[i], -0.5 - BULLET_MARGIN, 1e-9) << "axis " << i;
    EXPECT_NEAR(aabb_max[i], 0.5 + BULLET_MARGIN, 1e-9) << "axis " << i;
  }
}

/**
 * Building the Bullet shape of a convex mesh must be linear in the vertex count (issue #1392).
 * btConvexHullShape::addPoint() recalculates the local AABB over all points by default, which makes the build
 * quadratic. The vertex count grows 16 times here, so a linear build takes about 16 times longer and a quadratic
 * build at least 256 times longer. The bound of 64 leaves a factor of 4 on each side. Each time is the minimum of
 * several interleaved runs, which limits the effect of machine load on the ratio.
 */
TEST(TesseractConvexHullUnit, BulletConvexMeshBuildIsLinear)  // NOLINT
{
  const int small_count = 500;
  const int large_count = 16 * small_count;
  const auto small_vertices = spherePoints(small_count);
  const auto large_vertices = spherePoints(large_count);

  double small_seconds = std::numeric_limits<double>::max();
  double large_seconds = std::numeric_limits<double>::max();
  for (int i = 0; i < 10; ++i)
  {
    for (int j = 0; j < 4; ++j)
      small_seconds = std::min(small_seconds, convexMeshBuildSeconds(small_vertices));
    large_seconds = std::min(large_seconds, convexMeshBuildSeconds(large_vertices));
  }

  EXPECT_GT(small_seconds, 0.0);
  EXPECT_LT(large_seconds, 64.0 * small_seconds)
      << small_count << " vertices: " << small_seconds << " s, " << large_count << " vertices: " << large_seconds
      << " s, ratio " << large_seconds / small_seconds;
}
