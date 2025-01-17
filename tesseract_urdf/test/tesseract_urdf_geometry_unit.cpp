#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <octomap/OcTree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_geometry)  // NOLINT
{
  const bool global_make_convex = false;
  const auto parse_geometry_fn =
      [](const tinyxml2::XMLElement* xml_element, const tesseract_common::ResourceLocator& locator, const bool visual) {
        return tesseract_urdf::parseGeometry(xml_element, locator, visual, global_make_convex);
      };

  tesseract_common::GeneralResourceLocator resource_locator;
  {
    std::string str = R"(<geometry extra="0 0 0">
                           <box size="1 1 1" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::BOX);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sphere radius="1" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::SPHERE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cylinder radius="1" length="1" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::CYLINDER);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:cone radius="1" length="1" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::CONE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:capsule radius="1" length="1" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::CAPSULE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:octomap shape_type="box" extra="0 0 0">
                             <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt" extra="0 0 0"/>
                           </tesseract:octomap>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::OCTREE);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::MESH);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:sdf_mesh filename="package://tesseract_support/meshes/box_2m.ply" scale="1 2 1" extra="0 0 0"/>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem->getType() == tesseract_geometry::GeometryType::SDF_MESH);
  }

  {
    std::string str = R"(<geometry>
                           <unknown_type extra="0 0 0"/>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry>
                           <box size="1 1 a" />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <sphere />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <cylinder />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:cone />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:capsule />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:octomap shape_type="box" extra="0 0 0">
                             <tesseract:octree />
                           </tesseract:octomap>
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <mesh />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<geometry extra="0 0 0">
                           <tesseract:sdf_mesh />
                         </geometry>)";
    tesseract_geometry::Geometry::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_geometry::Geometry::Ptr>(
        elem, parse_geometry_fn, str, tesseract_urdf::GEOMETRY_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(elem == nullptr);
  }
}

TEST(TesseractURDFUnit, write_geometry)  // NOLINT
{
  // The catch clause for many of the subtypes is not triggered by these tests.  Triggering them would require sending
  // a nullptr to the write function - for now.  These could be expanded in the future to have more failure modes, and
  // already having the catch blocks in place seems wise.

  {  // trigger check for nullptr input
    tesseract_geometry::Geometry::Ptr geometry = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {  // sphere
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Sphere>(1.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_NE(text, "");
  }

  {  // cylinder
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Cylinder>(1.0, 1.414);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_NE(text, "");
  }

  {  // capsule
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Capsule>(1.0, 1.57);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_NE(text, "");
  }

  {  // cone
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Cone>(1.0, 2.3);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_NE(text, "");
  }

  {  // box
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Box>(1.0, 2.0, 3.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_NE(text, "");
  }

  {  // plane
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Plane>(1.0, 1.1, 1.2, 1.3);
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {  // mesh
    tesseract_common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                  Eigen::Vector3d(1, 0, 0),
                                                  Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Mesh>(
        std::make_shared<tesseract_common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));

    std::string text;
    EXPECT_EQ(
        0,
        writeTest<tesseract_geometry::Geometry::Ptr>(
            geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("geom0")));
    EXPECT_NE(text, "");
  }

  {  // sdf_mesh
    tesseract_common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                  Eigen::Vector3d(1, 0, 0),
                                                  Eigen::Vector3d(0, 1, 0) };
    Eigen::VectorXi indices(4);
    indices << 3, 0, 1, 2;
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::SDFMesh>(
        std::make_shared<tesseract_common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));

    std::string text;
    EXPECT_EQ(
        0,
        writeTest<tesseract_geometry::Geometry::Ptr>(
            geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("geom2")));
    EXPECT_NE(text, "");
  }

  {  // octree
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(
        0,
        writeTest<tesseract_geometry::Geometry::Ptr>(
            geometry, &tesseract_urdf::writeGeometry, text, tesseract_common::getTempPath(), std::string("geom3")));
    EXPECT_NE(text, "");
  }

  {  // octree failed-to-write
    tesseract_geometry::Geometry::Ptr geometry = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Geometry::Ptr>(
                  geometry, &tesseract_urdf::writeGeometry, text, std::string("/tmp/nonexistant/"), std::string("")));
    EXPECT_EQ(text, "");
  }
}
