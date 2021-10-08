#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/convex_mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_convex_mesh)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, true));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 12);
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" convert="true"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() >= 6);  // Because we are converting due to numerical variance you could end up
                                                // with additional faces.
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
  }

  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_box.dae" scale="1 2 1" convert="true"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.size() == 2);
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVertexCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="a 2 1" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 a 1" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 a" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="-1 2 1" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 -1 1" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 -1" />)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="abc" scale="1 2 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 a 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1 3"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = R"(<convex_mesh scale="1 2 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }

  {
    std::string str = "<convex_mesh />";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(
        geom, &tesseract_urdf::parseConvexMesh, str, "convex_mesh", resource_locator, 2, false));
    EXPECT_TRUE(geom.empty());
  }
}

TEST(TesseractURDFUnit, write_convex_mesh)  // NOLINT
{
  {
    tesseract_common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                  Eigen::Vector3d(1, 0, 0),
                                                  Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract_geometry::ConvexMesh::Ptr convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(
        std::make_shared<tesseract_common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));
    std::string text;
    EXPECT_EQ(
        0,
        writeTest<tesseract_geometry::ConvexMesh::Ptr>(
            convex_mesh, &tesseract_urdf::writeConvexMesh, text, std::string("/tmp/"), std::string("convex0.ply")));
    EXPECT_NE(text, "");
  }

  {
    tesseract_common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                  Eigen::Vector3d(1, 0, 0),
                                                  Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract_geometry::ConvexMesh::Ptr convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(
        std::make_shared<tesseract_common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::ConvexMesh::Ptr>(
                  convex_mesh, &tesseract_urdf::writeConvexMesh, text, std::string("/tmp/"), std::string("")));
    EXPECT_EQ(text, "");
  }

  {
    tesseract_geometry::ConvexMesh::Ptr convex_mesh = nullptr;
    std::string text;
    EXPECT_EQ(
        1,
        writeTest<tesseract_geometry::ConvexMesh::Ptr>(
            convex_mesh, &tesseract_urdf::writeConvexMesh, text, std::string("/tmp/"), std::string("convex1.ply")));
    EXPECT_EQ(text, "");
  }
}
