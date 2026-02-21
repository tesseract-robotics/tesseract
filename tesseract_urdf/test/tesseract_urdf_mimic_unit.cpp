#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/mimic.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_mimic)  // NOLINT
{
  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1" offset="2" extra="0 0 0"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" offset="2"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="a" offset="2"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1" offset="a"/>)";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<mimic />";
    tesseract::scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointMimic::Ptr>(
        elem, &tesseract::urdf::parseMimic, str, tesseract::urdf::MIMIC_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_mimic)  // NOLINT
{
  {
    tesseract::scene_graph::JointMimic::Ptr mimic = std::make_shared<tesseract::scene_graph::JointMimic>();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::scene_graph::JointMimic::Ptr>(mimic, &tesseract::urdf::writeMimic, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::scene_graph::JointMimic::Ptr mimic = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::scene_graph::JointMimic::Ptr>(mimic, &tesseract::urdf::writeMimic, text));
    EXPECT_EQ(text, "");
  }
}
