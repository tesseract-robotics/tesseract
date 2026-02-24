#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract/urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/urdf/material.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_material)  // NOLINT
{
  auto m = std::make_shared<tesseract::scene_graph::Material>("test_material");
  m->color = Eigen::Vector4d(1, .5, .5, 1);
  m->texture_filename = tesseract::common::getTempPath() + "texture.txt";

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material" extra="0 0 0">
                           <color rgba="1 .5 .5 1" extra="0 0 0"/>
                           <texture filename=")" +
                      m->texture_filename + R"("extra="0 0 0"/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                               &tesseract::urdf::parseMaterial,
                                                               str,
                                                               tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                               empty_available_materials,
                                                               true));
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == tesseract::common::getTempPath() + "texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1"/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                               &tesseract::urdf::parseMaterial,
                                                               str,
                                                               tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                               empty_available_materials,
                                                               true));
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename.empty());
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material"/>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                               &tesseract::urdf::parseMaterial,
                                                               str,
                                                               tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                               available_materials,
                                                               true));
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == tesseract::common::getTempPath() + "texture.txt");
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material" extra="0 0 0">
                           <color rgba="1 .5 .5 1" extra="0 0 0"/>
                           <texture />
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color />
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material />)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material"/>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 a"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 a 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 a .5 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="a .5 .5 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba="1 .5 .5 1 1"/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }

  {
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> empty_available_materials;
    std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr> available_materials;
    available_materials["test_material"] = m;

    std::string str = R"(<material name="test_material">
                           <color rgba=""/>
                           <texture filename=")" +
                      m->texture_filename + R"("/>
                         </material>)";
    tesseract::scene_graph::Material::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Material::Ptr>(elem,
                                                                &tesseract::urdf::parseMaterial,
                                                                str,
                                                                tesseract::urdf::MATERIAL_ELEMENT_NAME.data(),
                                                                empty_available_materials,
                                                                true));
  }
}

TEST(TesseractURDFUnit, write_material)  // NOLINT
{
  {
    tesseract::scene_graph::Material::Ptr material = std::make_shared<tesseract::scene_graph::Material>("unobtainium");
    material->color = Eigen::Vector4d(1.0, 0.5, 0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::scene_graph::Material::Ptr>(material, &tesseract::urdf::writeMaterial, text));
    // This string literal is not pretty, but has to match whitespace for comparison.
    // clang-format off
    std::string expected =
R"(<material name="unobtainium">
    <color rgba="1 0.5 0.5 1"/>
</material>)";
    // clang-format on
    EXPECT_EQ(text, expected);
  }

  {
    tesseract::scene_graph::Material::Ptr material = std::make_shared<tesseract::scene_graph::Material>("unobtainium");
    material->color = Eigen::Vector4d(1.0, 0.5, 0.5, 1.0);
    material->texture_filename = "/tmp/texture.txt";
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::scene_graph::Material::Ptr>(material, &tesseract::urdf::writeMaterial, text));
    // This string literal is not pretty, but has to match whitespace for comparison.
    // clang-format off
    std::string expected =
R"(<material name="unobtainium">
    <texture filename="/tmp/texture.txt"/>
    <color rgba="1 0.5 0.5 1"/>
</material>)";
    // clang-format on
    EXPECT_EQ(text, expected);
  }
  {
    tesseract::scene_graph::Material::Ptr material = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::scene_graph::Material::Ptr>(material, &tesseract::urdf::writeMaterial, text));
    EXPECT_EQ(text, "");
  }
}
