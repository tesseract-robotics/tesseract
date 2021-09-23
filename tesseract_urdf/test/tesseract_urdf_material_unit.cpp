#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_urdf/material.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_material)  // NOLINT
{
  auto m = std::make_shared<tesseract_scene_graph::Material>("test_material");
  m->color = Eigen::Vector4d(1, .5, .5, 1);
  m->texture_filename = tesseract_common::getTempPath() + "texture.txt";

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material" extra="0 0 0">
                           <color rgba="1 .5 .5 1" extra="0 0 0"/>
                           <texture filename=")" +
                      m->texture_filename + R"("extra="0 0 0"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == tesseract_common::getTempPath() + "texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1"/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
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
    EXPECT_TRUE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", available_materials, 2, true));
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == tesseract_common::getTempPath() + "texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material" extra="0 0 0">
                           <color rgba="1 .5 .5 1" extra="0 0 0"/>
                           <texture />
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color />
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material />)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material"/>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 a"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 a 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 a .5 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="a .5 .5 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba=""/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract_scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Material::Ptr>(
        elem, &tesseract_urdf::parseMaterial, str, "material", empty_available_materials, 2, true));
  }
}

TEST(TesseractURDFUnit, write_material)  // NOLINT
{
  {
    tesseract_scene_graph::Material::Ptr material = std::make_shared<tesseract_scene_graph::Material>("unobtainium");
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Material::Ptr>(material, &tesseract_urdf::writeMaterial, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract_scene_graph::Material::Ptr material = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::Material::Ptr>(material, &tesseract_urdf::writeMaterial, text));
    EXPECT_EQ(text, "");
  }
}
