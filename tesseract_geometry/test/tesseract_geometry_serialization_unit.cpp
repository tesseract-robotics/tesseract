/**
 * @file tesseract_geometry_serialization_unit.cpp
 * @brief Tests serialization of geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_geometry/impl/octree_utils.h>

using namespace tesseract_geometry;

TEST(TesseractGeometrySerializeUnit, Box)  // NOLINT
{
  auto object = std::make_shared<Box>(1, 2, 3);
  tesseract_common::testSerialization<Box>(*object, "Box");
  tesseract_common::testSerializationDerivedClass<Geometry, Box>(object, "Box");
}

TEST(TesseractGeometrySerializeUnit, Capsule)  // NOLINT
{
  auto object = std::make_shared<Capsule>(1, 2);
  tesseract_common::testSerialization<Capsule>(*object, "Capsule");
  tesseract_common::testSerializationDerivedClass<Geometry, Capsule>(object, "Capsule");
}

TEST(TesseractGeometrySerializeUnit, Cone)  // NOLINT
{
  auto object = std::make_shared<Cone>(1.1, 2.2);
  tesseract_common::testSerialization<Cone>(*object, "Cone");
  tesseract_common::testSerializationDerivedClass<Geometry, Cone>(object, "Cone");
}

TEST(TesseractGeometrySerializeUnit, ConvexMesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract_common::testSerialization<ConvexMesh>(*object.front(), "ConvexMesh");
  tesseract_common::testSerializationDerivedClass<Geometry, ConvexMesh>(object.front(), "ConvexMesh");
  tesseract_common::testSerializationDerivedClass<PolygonMesh, ConvexMesh>(object.front(), "ConvexMesh");
}

TEST(TesseractGeometrySerializeUnit, CompoundConvexMesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  std::vector<tesseract_geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(object.front());
  meshes.push_back(object.front());
  meshes.push_back(object.front());

  auto compound_object = std::make_shared<CompoundMesh>(meshes);
  tesseract_common::testSerialization<CompoundMesh>(*compound_object, "CompundConvexMesh");
  tesseract_common::testSerializationDerivedClass<Geometry, CompoundMesh>(compound_object, "CompundConvexMesh");
}

TEST(TesseractGeometrySerializeUnit, Cylinder)  // NOLINT
{
  auto object = std::make_shared<Cylinder>(3.3, 4.4);
  tesseract_common::testSerialization<Cylinder>(*object, "Cylinder");
  tesseract_common::testSerializationDerivedClass<Geometry, Cylinder>(object, "Cylinder");
}

TEST(TesseractGeometrySerializeUnit, Mesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract_common::testSerialization<Mesh>(*object.front(), "Mesh");
  tesseract_common::testSerializationDerivedClass<Geometry, Mesh>(object.front(), "Mesh");
  tesseract_common::testSerializationDerivedClass<PolygonMesh, Mesh>(object.front(), "Mesh");
}

TEST(TesseractGeometrySerializeUnit, CompoundMesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  std::vector<tesseract_geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(object.front());
  meshes.push_back(object.front());
  meshes.push_back(object.front());

  auto compound_object = std::make_shared<CompoundMesh>(meshes);
  tesseract_common::testSerialization<CompoundMesh>(*compound_object, "CompoundMesh");
  tesseract_common::testSerializationDerivedClass<Geometry, CompoundMesh>(compound_object, "CompoundMesh");
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
    auto octree = tesseract_geometry::createOctree(pc, 1, false, true);
    auto object = std::make_shared<tesseract_geometry::Octree>(
        std::move(octree), tesseract_geometry::OctreeSubType::BOX, false, true);
    tesseract_common::testSerialization<Octree>(*object, "Binary_Octree");
    tesseract_common::testSerializationDerivedClass<Geometry, Octree>(object, "Binary_Octree");
  }
  {
    auto octree = tesseract_geometry::createOctree(pc, 1, false, false);
    auto object = std::make_shared<tesseract_geometry::Octree>(
        std::move(octree), tesseract_geometry::OctreeSubType::BOX, false, false);
    tesseract_common::testSerialization<Octree>(*object, "Full_Octree");
    tesseract_common::testSerializationDerivedClass<Geometry, Octree>(object, "Full_Octree");
  }
}

TEST(TesseractGeometrySerializeUnit, Plane)  // NOLINT
{
  auto object = std::make_shared<Plane>(1.1, 2, 3.3, 4);
  tesseract_common::testSerialization<Plane>(*object, "Plane");
  tesseract_common::testSerializationDerivedClass<Geometry, Plane>(object, "Plane");
}

TEST(TesseractGeometrySerializeUnit, PolygonMesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::PolygonMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract_common::testSerialization<PolygonMesh>(*object.front(), "PolygonMesh");
  tesseract_common::testSerializationDerivedClass<Geometry, PolygonMesh>(object.front(), "PolygonMesh");
}

TEST(TesseractGeometrySerializeUnit, SDFMesh)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string path = "package://tesseract_support/meshes/sphere_p25m.stl";
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::SDFMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  tesseract_common::testSerialization<SDFMesh>(*object.front(), "SDFMesh");
  tesseract_common::testSerializationDerivedClass<Geometry, SDFMesh>(object.front(), "SDFMesh");
  tesseract_common::testSerializationDerivedClass<PolygonMesh, SDFMesh>(object.front(), "SDFMesh");
}

TEST(TesseractGeometrySerializeUnit, Sphere)  // NOLINT
{
  auto object = std::make_shared<Sphere>(3.3);
  tesseract_common::testSerialization<Sphere>(*object, "Sphere");
  tesseract_common::testSerializationDerivedClass<Geometry, Sphere>(object, "Sphere");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
