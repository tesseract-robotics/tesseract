#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/convex_mesh.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_convex_mesh)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" convert="true"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() >= 6);  // Because we are converting due to numerical variance you could end up
                                                // with additional faces.
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
  }

  {
    std::string str =
        R"(<convex_mesh filename="package://tesseract_support/meshes/box_box.dae" scale="1 2 1" convert="true"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 2);
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = R"(<convex_mesh filename="abc" scale="1 2 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 a 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1 3"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<convex_mesh scale="1 2 1"/>)";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<convex_mesh />";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status =
        runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", resource_locator, 2, false);
    EXPECT_FALSE(*status);
  }
}
