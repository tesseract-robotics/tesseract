#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/visual.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_visual)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
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
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, str, "visual", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, str, "visual", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, str, "visual", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(
        elem, str, "visual", resource_locator, empty_available_materials, 2);
    EXPECT_FALSE(*status);
  }
}
