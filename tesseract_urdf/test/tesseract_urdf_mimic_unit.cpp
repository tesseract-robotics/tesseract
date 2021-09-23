#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/mimic.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_mimic)  // NOLINT
{
  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1" offset="2" extra="0 0 0"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" offset="2"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="a" offset="2"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1" offset="a"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
  }

  {
    std::string str = "<mimic />";
    tesseract_scene_graph::JointMimic::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, &tesseract_urdf::parseMimic, str, "mimic", 2));
  }
}

TEST(TesseractURDFUnit, write_mimic)  // NOLINT
{
  {
    tesseract_scene_graph::JointMimic::Ptr mimic = std::make_shared<tesseract_scene_graph::JointMimic>();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointMimic::Ptr>(mimic, &tesseract_urdf::writeMimic, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract_scene_graph::JointMimic::Ptr mimic = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::JointMimic::Ptr>(mimic, &tesseract_urdf::writeMimic, text));
    EXPECT_EQ(text, "");
  }
}
