#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/geometry.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_geometry)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  {
    std::string str = R"(<geometry extra="0 0 0">
                           <box size="1 1 1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::BOX);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sphere radius="1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::SPHERE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cylinder radius="1" length="1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::CYLINDER);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cone radius="1" length="1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::CONE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <capsule radius="1" length="1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::CAPSULE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <octomap shape_type="box" extra="0 0 0">
                             <octree filename="package://tesseract_support/meshes/box_2m.bt" extra="0 0 0"/>
                           </octomap>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::OCTREE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <convex_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::CONVEX_MESH);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::MESH);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sdf_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::SDF_MESH);
  }

  {
    std::string str = R"(<geometry>
                           <unknown_type extra="0 0 0"/>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry>
                           <box size="1 1 a" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sphere />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cylinder />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cone />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <capsule />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <octomap shape_type="box" extra="0 0 0">
                             <octree />
                           </octomap>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <convex_mesh />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <mesh />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sdf_mesh />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(
        elem, &tesseract_urdf::parseGeometry, str, "geometry", resource_locator, 2, true));
    EXPECT_TRUE(elem.empty());
  }
}
