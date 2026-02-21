#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/mesh.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

static std::string getTempPkgPath()
{
  std::string tmp = tesseract::common::getTempPath();
  std::string tmppkg = tmp + "tmppkg";
  if (!std::filesystem::is_directory(tmppkg) || !std::filesystem::exists(tmppkg))
  {
    std::filesystem::create_directory(tmppkg);
  }
  return tmppkg;
}

TEST(TesseractURDFUnit, parse_mesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  const bool global_make_convex = false;
  const auto parse_mesh_fn =
      [&](const tinyxml2::XMLElement* xml_element, const tesseract::common::ResourceLocator& locator, bool visual) {
        return tesseract::urdf::parseMesh(xml_element, locator, visual, global_make_convex);
      };

  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" scale="1 2 1" extra="0 0 0"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 80);
    EXPECT_EQ(geom[0]->getVertexCount(), 240);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 80);
    EXPECT_EQ(geom[0]->getVertexCount(), 240);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  // Make convex override false (correct)
  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" tesseract:make_convex="false"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 80);
    EXPECT_EQ(geom[0]->getVertexCount(), 240);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  // Make convex override true (correct)
  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" tesseract:make_convex="true"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_LE(geom[0]->getFaceCount(), 80);
    EXPECT_LE(geom[0]->getVertexCount(), 240);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  // Make convex (incorrect)
  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" tesseract:make_convex="foo"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="abc" scale="1 2 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" scale="1 a 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" scale="a 1 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" scale="1 1 a"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/sphere_p25m.stl" scale="1 2 1 3"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh scale="1 2 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = "<mesh />";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom.empty());
  }
}

TEST(TesseractURDFUnit, write_mesh)  // NOLINT
{
  {
    tesseract::common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                   Eigen::Vector3d(1, 0, 0),
                                                   Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract::geometry::Mesh::Ptr mesh = std::make_shared<tesseract::geometry::Mesh>(
        std::make_shared<tesseract::common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::geometry::Mesh::Ptr>(
                  mesh, &tesseract::urdf::writeMesh, text, getTempPkgPath(), std::string("mesh0.ply")));
    EXPECT_EQ(text, R"(<mesh filename="package://tmppkg/mesh0.ply"/>)");
  }

  {  // fail to write
    tesseract::common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                   Eigen::Vector3d(1, 0, 0),
                                                   Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract::geometry::Mesh::Ptr mesh = std::make_shared<tesseract::geometry::Mesh>(
        std::make_shared<tesseract::common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::geometry::Mesh::Ptr>(
                  mesh, &tesseract::urdf::writeMesh, text, tesseract::common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {
    tesseract::geometry::Mesh::Ptr mesh = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::geometry::Mesh::Ptr>(
                  mesh, &tesseract::urdf::writeMesh, text, tesseract::common::getTempPath(), std::string("mesh1.ply")));
    EXPECT_EQ(text, "");
  }
}

TEST(TesseractURDFUnit, parse_convex_mesh)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  const bool global_make_convex = true;
  const auto parse_mesh_fn =
      [&](const tinyxml2::XMLElement* xml_element, const tesseract::common::ResourceLocator& locator, bool visual) {
        return tesseract::urdf::parseMesh(xml_element, locator, visual, global_make_convex);
      };

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 7);
    EXPECT_EQ(geom[0]->getVertexCount(), 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  // Global and local override set to make convex mesh
  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0" tesseract:make_convex="true"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 7);
    EXPECT_EQ(geom[0]->getVertexCount(), 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  // Global and local override set to make convex mesh
  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0" tesseract:make_convex="false"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_EQ(geom[0]->getFaceCount(), 12);
    EXPECT_EQ(geom[0]->getVertexCount(), 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" tesseract:make_convex="true"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() >= 6);  // Because we are converting due to numerical variance you could end up
                                                // with additional faces.
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
  }

  {
    std::string str =
        R"(<mesh filename="package://tesseract_support/meshes/box_box.dae" scale="1 2 1" tesseract:make_convex="true"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 2);
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 7);
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="a 2 1" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 a 1" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 a" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="-1 2 1" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 -1 1" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 -1" />)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="abc" scale="1 2 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 a 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1 3"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<mesh scale="1 2 1"/>)";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = "<mesh />";
    std::vector<tesseract::geometry::PolygonMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract::geometry::PolygonMesh::Ptr>>(
        geom, parse_mesh_fn, str, tesseract::urdf::MESH_ELEMENT_NAME.data(), resource_locator, false));
    EXPECT_TRUE(geom.empty());
  }
}

TEST(TesseractURDFUnit, write_convex_mesh)  // NOLINT
{
  {
    // Create an arbitrary mesh denoted specifically as a convex mesh type
    tesseract::common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                   Eigen::Vector3d(1, 0, 0),
                                                   Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    auto convex_mesh = std::make_shared<tesseract::geometry::ConvexMesh>(
        std::make_shared<tesseract::common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));

    // Write the convex mesh into a string
    // Since the input type is specifically a ConvexMesh, the tesseract:make_convex` attribute should be present to
    // indicate that mesh should be made convex and produce a `ConvexMesh`
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::geometry::ConvexMesh::Ptr>(
                  convex_mesh, &tesseract::urdf::writeMesh, text, getTempPkgPath(), std::string("convex0.ply")));
    EXPECT_EQ(text, R"(<mesh filename="package://tmppkg/convex0.ply" tesseract:make_convex="true"/>)");
  }

  {  // With scale
    // Create an arbitrary mesh denoted specifically as a convex mesh type
    tesseract::common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                   Eigen::Vector3d(1, 0, 0),
                                                   Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    Eigen::Vector3d scale(0.5, 0.5, 0.5);
    auto convex_mesh =
        std::make_shared<tesseract::geometry::ConvexMesh>(std::make_shared<tesseract::common::VectorVector3d>(vertices),
                                                          std::make_shared<Eigen::VectorXi>(indices),
                                                          nullptr,
                                                          scale);

    // Write the convex mesh into a string
    // Since the input type is specifically a ConvexMesh, the tesseract:make_convex` attribute should be present to
    // indicate that mesh should be made convex and produce a `ConvexMesh`
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::geometry::ConvexMesh::Ptr>(
                  convex_mesh, &tesseract::urdf::writeMesh, text, getTempPkgPath(), std::string("convex1.ply")));
    EXPECT_EQ(text,
              R"(<mesh filename="package://tmppkg/convex1.ply" scale="0.5 0.5 0.5" tesseract:make_convex="true"/>)");
  }
}
