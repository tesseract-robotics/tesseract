#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/material.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_material)  // NOLINT
{
  auto m = std::make_shared<tesseract_scene_graph::Material>("test_material");
  m->color = Eigen::Vector4d(1, .5, .5, 1);
  m->texture_filename = "/tmp/texture.txt";

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material" extra="0 0 0">
                           <color rgba="1 .5 .5 1" extra="0 0 0"/>
                           <texture filename="/tmp/texture.txt" extra="0 0 0"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials, 2, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == "/tmp/texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials, 2, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename.empty());
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material"/>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", available_materials, 2, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == "/tmp/texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material"/>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials, 2, true);
    EXPECT_FALSE(*status);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 a"/>
                           <texture filename="/tmp/texture.txt"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials, 2, true);
    EXPECT_FALSE(*status);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1 1"/>
                           <texture filename="/tmp/texture.txt"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials, 2, true);
    EXPECT_FALSE(*status);
  }
}
