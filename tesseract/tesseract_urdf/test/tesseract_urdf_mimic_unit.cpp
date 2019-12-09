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
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" offset="2"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="a" offset="2"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<mimic joint="joint_1" multiplier="1" offset="a"/>)";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mimic />";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic", 2);
    EXPECT_FALSE(*status);
  }
}
