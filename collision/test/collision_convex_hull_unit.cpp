#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <system_error>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/common/ply_io.h>
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
  std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("tesseract-convex-hull-" + boost::uuids::to_string(boost::uuids::random_generator()()) + ".ply");
  ~PlyFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};
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
