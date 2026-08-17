#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/compound_mesh.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/urdf/box.h>
#include <tesseract/urdf/visual.h>
#include <tesseract/common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

/** @brief Creates the package directory and the visual sub-directory writeVisual expects to exist */
static std::string getTempVisualPkgPath()
{
  std::filesystem::path pkg_path = std::filesystem::path(tesseract::common::getTempPath()) / "tmppkg";
  std::filesystem::create_directories(pkg_path / "visual");
  return pkg_path.string();
}

TEST(TesseractURDFUnit, parse_visual)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual extra="0 0 0">
                           <origin xyz="1 2 3" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    tesseract::scene_graph::Visual::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Visual::Ptr>(elem,
                                                             &tesseract::urdf::parseVisual,
                                                             str,
                                                             tesseract::urdf::VISUAL_ELEMENT_NAME.data(),
                                                             resource_locator,
                                                             empty_available_materials));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_TRUE(elem->material != nullptr);
    EXPECT_FALSE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    tesseract::scene_graph::Visual::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Visual::Ptr>(elem,
                                                             &tesseract::urdf::parseVisual,
                                                             str,
                                                             tesseract::urdf::VISUAL_ELEMENT_NAME.data(),
                                                             resource_locator,
                                                             empty_available_materials));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_TRUE(elem->material != nullptr);
    EXPECT_TRUE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </visual>)";
    tesseract::scene_graph::Visual::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Visual::Ptr>(elem,
                                                             &tesseract::urdf::parseVisual,
                                                             str,
                                                             tesseract::urdf::VISUAL_ELEMENT_NAME.data(),
                                                             resource_locator,
                                                             empty_available_materials));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_TRUE(elem->material != nullptr);
    EXPECT_TRUE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    tesseract::scene_graph::Visual::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Visual::Ptr>(elem,
                                                              &tesseract::urdf::parseVisual,
                                                              str,
                                                              tesseract::urdf::VISUAL_ELEMENT_NAME.data(),
                                                              resource_locator,
                                                              empty_available_materials));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box />
                           </geometry>
                         </visual>)";
    tesseract::scene_graph::Visual::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Visual::Ptr>(elem,
                                                              &tesseract::urdf::parseVisual,
                                                              str,
                                                              tesseract::urdf::VISUAL_ELEMENT_NAME.data(),
                                                              resource_locator,
                                                              empty_available_materials));
  }
}

TEST(TesseractURDFUnit, write_visual)  // NOLINT
{
  {  // trigger check for an assigned name and check for specified ID
    tesseract::scene_graph::Visual::Ptr visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Box>(1.0, 1.0, 1.0);
    visual->material = std::make_shared<tesseract::scene_graph::Material>("black");
    std::string text;
    EXPECT_EQ(
        0,
        writeTest<tesseract::scene_graph::Visual::Ptr>(
            visual, &tesseract::urdf::writeVisual, text, tesseract::common::getTempPath(), std::string("test"), 0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for nullptr input
    tesseract::scene_graph::Visual::Ptr visual = nullptr;
    std::string text;
    EXPECT_EQ(
        1,
        writeTest<tesseract::scene_graph::Visual::Ptr>(
            visual, &tesseract::urdf::writeVisual, text, tesseract::common::getTempPath(), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }

  {  // trigger check for bad geometry
    tesseract::scene_graph::Visual::Ptr visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = nullptr;
    std::string text;
    EXPECT_EQ(
        1,
        writeTest<tesseract::scene_graph::Visual::Ptr>(
            visual, &tesseract::urdf::writeVisual, text, tesseract::common::getTempPath(), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }
}

TEST(TesseractURDFUnit, WriteVisualMeshFilenameUsesTheVisualName)  // NOLINT
{
  tesseract::common::VectorVector3d vertices = { Eigen::Vector3d(0, 0, 0),
                                                 Eigen::Vector3d(1, 0, 0),
                                                 Eigen::Vector3d(0, 1, 0) };
  Eigen::VectorXi indices(4);
  indices << 3, 0, 1, 2;

  auto visual = std::make_shared<tesseract::scene_graph::Visual>();
  visual->name = "camera_mount";
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract::geometry::Mesh>(
      std::make_shared<tesseract::common::VectorVector3d>(vertices), std::make_shared<Eigen::VectorXi>(indices));

  std::string text;
  EXPECT_EQ(0,
            writeTest<tesseract::scene_graph::Visual::Ptr>(
                visual, &tesseract::urdf::writeVisual, text, getTempVisualPkgPath(), std::string("base_link"), 0));
  EXPECT_NE(text.find(R"(filename="package://tmppkg/visual/base_link_camera_mount_0.ply")"), std::string::npos)
      << "emitted: " << text;
}
