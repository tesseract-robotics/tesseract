#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/box.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/visual.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_visual)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual extra="0 0 0">
                           <origin xyz="1 2 3" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, &tesseract_urdf::parseVisual, str, "visual", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_FALSE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, &tesseract_urdf::parseVisual, str, "visual", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </visual>)";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, &tesseract_urdf::parseVisual, str, "visual", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <material name="Cyan">
                             <color rgba="0 1.0 1.0 1.0"/>
                           </material>
                         </visual>)";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, &tesseract_urdf::parseVisual, str, "visual", resource_locator, empty_available_materials, 2));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<visual>
                           <geometry>
                             <box />
                           </geometry>
                         </visual>)";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, &tesseract_urdf::parseVisual, str, "visual", resource_locator, empty_available_materials, 2));
  }
}

TEST(TesseractURDFUnit, write_visual)  // NOLINT
{
  {  // trigger check for an assigned name and check for specified ID
    tesseract_scene_graph::Visual::Ptr visual = std::make_shared<tesseract_scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    visual->material = std::make_shared<tesseract_scene_graph::Material>("black");
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_scene_graph::Visual::Ptr>(
                  visual, &tesseract_urdf::writeVisual, text, std::string("/tmp/"), std::string("test"), 0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for nullptr input
    tesseract_scene_graph::Visual::Ptr visual = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_scene_graph::Visual::Ptr>(
                  visual, &tesseract_urdf::writeVisual, text, std::string("/tmp/"), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }

  {  // trigger check for bad geometry
    tesseract_scene_graph::Visual::Ptr visual = std::make_shared<tesseract_scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_scene_graph::Visual::Ptr>(
                  visual, &tesseract_urdf::writeVisual, text, std::string("/tmp/"), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }
}
