/**
 * @file tesseract_geometry_serialization_unit.cpp
 * @brief Tests serialization of geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <sstream>
#include <octomap/octomap.h>
#include <cereal/archives/xml.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/geometry/cereal_serialization.h>
#include <tesseract/geometry/impl/octree_utils.h>
#include <tesseract/common/serialization.h>

using namespace tesseract::geometry;

TEST(TesseractGeometrySerializeUnit, Box)  // NOLINT
{
  auto object = std::make_shared<Box>(1, 2, 3);
  tesseract::common::testSerialization<Box>(*object, "Box");
  tesseract::common::testSerializationDerivedClass<Geometry, Box>(object, "Box");
}

TEST(TesseractGeometrySerializeUnit, Capsule)  // NOLINT
{
  auto object = std::make_shared<Capsule>(1, 2);
  tesseract::common::testSerialization<Capsule>(*object, "Capsule");
  tesseract::common::testSerializationDerivedClass<Geometry, Capsule>(object, "Capsule");
}

TEST(TesseractGeometrySerializeUnit, Cone)  // NOLINT
{
  auto object = std::make_shared<Cone>(1.1, 2.2);
  tesseract::common::testSerialization<Cone>(*object, "Cone");
  tesseract::common::testSerializationDerivedClass<Geometry, Cone>(object, "Cone");
}

TEST(TesseractGeometrySerializeUnit, ConvexMesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = "package://tesseract/support/meshes/sphere_p25m.stl";
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::ConvexMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract::common::testSerialization<ConvexMesh>(*object.front(), "ConvexMesh");
  tesseract::common::testSerializationDerivedClass<Geometry, ConvexMesh>(object.front(), "ConvexMesh");
  tesseract::common::testSerializationDerivedClass<PolygonMesh, ConvexMesh>(object.front(), "ConvexMesh");
}

TEST(TesseractGeometrySerializeUnit, CompoundConvexMesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = "package://tesseract/support/meshes/sphere_p25m.stl";
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::ConvexMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  std::vector<tesseract::geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(object.front());
  meshes.push_back(object.front());
  meshes.push_back(object.front());

  auto compound_object = std::make_shared<CompoundMesh>(meshes);
  tesseract::common::testSerialization<CompoundMesh>(*compound_object, "CompundConvexMesh");
  tesseract::common::testSerializationDerivedClass<Geometry, CompoundMesh>(compound_object, "CompundConvexMesh");
}

TEST(TesseractGeometrySerializeUnit, Cylinder)  // NOLINT
{
  auto object = std::make_shared<Cylinder>(3.3, 4.4);
  tesseract::common::testSerialization<Cylinder>(*object, "Cylinder");
  tesseract::common::testSerializationDerivedClass<Geometry, Cylinder>(object, "Cylinder");
}

TEST(TesseractGeometrySerializeUnit, Mesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = "package://tesseract/support/meshes/sphere_p25m.stl";
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract::common::testSerialization<Mesh>(*object.front(), "Mesh");
  tesseract::common::testSerializationDerivedClass<Geometry, Mesh>(object.front(), "Mesh");
  tesseract::common::testSerializationDerivedClass<PolygonMesh, Mesh>(object.front(), "Mesh");
}

TEST(TesseractGeometrySerializeUnit, CompoundMesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = "package://tesseract/support/meshes/sphere_p25m.stl";
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  std::vector<tesseract::geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(object.front());
  meshes.push_back(object.front());
  meshes.push_back(object.front());

  auto compound_object = std::make_shared<CompoundMesh>(meshes);
  tesseract::common::testSerialization<CompoundMesh>(*compound_object, "CompoundMesh");
  tesseract::common::testSerializationDerivedClass<Geometry, CompoundMesh>(compound_object, "CompoundMesh");
}

TEST(TesseractGeometrySerializeUnit, Octree)  // NOLINT
{
  struct TestPointCloud
  {
    struct point
    {
      point(double x, double y, double z) : x(x), y(y), z(z) {}
      double x;
      double y;
      double z;
    };

    std::vector<point> points;
  };

  TestPointCloud pc;
  pc.points.emplace_back(.5, 0.5, 0.5);
  pc.points.emplace_back(-.5, -0.5, -0.5);
  pc.points.emplace_back(-.5, 0.5, 0.5);
  {
    auto octree = tesseract::geometry::createOctree(pc, 1, false, true);
    auto object = std::make_shared<tesseract::geometry::Octree>(
        std::move(octree), tesseract::geometry::OctreeSubType::BOX, false, true);

    // tesseract::common::testSerialization<tesseract::geometry::Octree>(*object, "Binary_Octree");
    tesseract::common::testSerializationDerivedClass<Geometry, Octree>(object, "Binary_Octree");
  }
  {
    auto octree = tesseract::geometry::createOctree(pc, 1, false, false);
    auto object = std::make_shared<tesseract::geometry::Octree>(
        std::move(octree), tesseract::geometry::OctreeSubType::BOX, false, false);
    // tesseract::common::testSerialization<Octree>(*object, "Full_Octree");
    tesseract::common::testSerializationDerivedClass<Geometry, Octree>(object, "Full_Octree");
  }
}

TEST(TesseractGeometrySerializeUnit, OctreeCloneBinaryFlag)  // NOLINT
{
  // binary_octree_ has no public getter, but it controls whether cereal
  // emits the OcTree payload via writeBinary (compact) or write (text).
  // A clone that drops binary_octree_ produces a different serialized
  // blob than the original.
  struct TestPointCloud
  {
    struct point
    {
      point(double x, double y, double z) : x(x), y(y), z(z) {}
      double x;
      double y;
      double z;
    };
    std::vector<point> points;
  };

  TestPointCloud pc;
  pc.points.emplace_back(.5, 0.5, 0.5);
  pc.points.emplace_back(-.5, -0.5, -0.5);
  pc.points.emplace_back(-.5, 0.5, 0.5);

  auto octree = createOctree(pc, 1, false, true);
  auto orig = std::make_shared<Octree>(std::move(octree), OctreeSubType::BOX, /*pruned=*/false, /*binary_octree=*/true);
  auto cloned = std::static_pointer_cast<Octree>(orig->clone());
  cloned->setUUID(orig->getUUID());

  auto orig_str = tesseract::common::Serialization::toArchiveStringXML<std::shared_ptr<Octree>>(orig, "octree");
  auto clone_str = tesseract::common::Serialization::toArchiveStringXML<std::shared_ptr<Octree>>(cloned, "octree");
  EXPECT_EQ(orig_str, clone_str);
}

TEST(TesseractGeometrySerializeUnit, Plane)  // NOLINT
{
  auto object = std::make_shared<Plane>(1.1, 2, 3.3, 4);
  tesseract::common::testSerialization<Plane>(*object, "Plane");
  tesseract::common::testSerializationDerivedClass<Geometry, Plane>(object, "Plane");
}

TEST(TesseractGeometrySerializeUnit, PolygonMesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  std::string path = "package://tesseract/support/meshes/sphere_p25m.stl";
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::PolygonMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract::common::testSerialization<PolygonMesh>(*object.front(), "PolygonMesh");
  tesseract::common::testSerializationDerivedClass<Geometry, PolygonMesh>(object.front(), "PolygonMesh");
}

TEST(TesseractGeometrySerializeUnit, SignedDistanceField)  // NOLINT
{
  const Eigen::AlignedBox3d domain(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  const Eigen::Vector3i dims(2, 2, 2);
  const std::vector<double> distances{ -0.5, -0.4, -0.3, -0.2, 0.1, 0.2, 0.3, 0.4 };
  auto object = std::make_shared<SignedDistanceField>(domain, dims, distances, Eigen::Vector3d(1.0, 2.0, 3.0));
  tesseract::common::testSerialization<SignedDistanceField>(*object, "SignedDistanceField");
  tesseract::common::testSerializationDerivedClass<Geometry, SignedDistanceField>(object, "SignedDistanceField");
}

TEST(TesseractGeometrySerializeUnit, SignedDistanceFieldRejectsMalformedArchive)  // NOLINT
{
  // The load path writes directly into the members (bypassing the validating constructor), then
  // re-runs the grid invariant. A malformed archive whose distances no longer match its dimensions
  // must be rejected rather than producing a field that later out-of-bounds in getDistance().
  const Eigen::AlignedBox3d domain(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  const Eigen::Vector3i dims(2, 2, 2);
  const std::vector<double> distances{ -0.5, -0.4, -0.3, -0.2, 0.1, 0.2, 0.3, 0.4 };
  SignedDistanceField object(domain, dims, distances);

  std::stringstream ss;
  {
    cereal::XMLOutputArchive ar(ss);
    ar(object);
  }
  const std::string xml = ss.str();

  // Sanity: the unmodified archive loads fine.
  {
    std::stringstream valid(xml);
    cereal::XMLInputArchive ar(valid);
    SignedDistanceField loaded;
    EXPECT_NO_THROW(ar(loaded));  // NOLINT
  }

  // Corrupt the grid so distances.size() (8) no longer matches dimensions.prod():
  // bump dim_x from 2 to 3 -> expected 3 * 2 * 2 = 12.
  std::string corrupted_xml = xml;
  const std::string from = "<dim_x>2</dim_x>";
  const std::string to = "<dim_x>3</dim_x>";
  const auto pos = corrupted_xml.find(from);
  ASSERT_NE(pos, std::string::npos);
  corrupted_xml.replace(pos, from.size(), to);

  std::stringstream corrupted(corrupted_xml);
  cereal::XMLInputArchive ar(corrupted);
  SignedDistanceField loaded;
  EXPECT_ANY_THROW(ar(loaded));  // NOLINT
}

TEST(TesseractGeometrySerializeUnit, Sphere)  // NOLINT
{
  auto object = std::make_shared<Sphere>(3.3);
  tesseract::common::testSerialization<Sphere>(*object, "Sphere");
  tesseract::common::testSerializationDerivedClass<Geometry, Sphere>(object, "Sphere");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
