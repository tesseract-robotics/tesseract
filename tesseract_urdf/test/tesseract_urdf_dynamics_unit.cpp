#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/dynamics.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_dynamics)  // NOLINT
{
  {
    std::string str = R"(<dynamics damping="1" friction="2" extra="0 0 0"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="1"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 0, 1e-8);
  }

  {
    std::string str = R"(<dynamics friction="2"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
    EXPECT_NEAR(elem->damping, 0, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="a" friction="2"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
  }

  {
    std::string str = R"(<dynamics damping="1" friction="b"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
  }

  {
    std::string str = "<dynamics/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(
        runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, &tesseract_urdf::parseDynamics, str, "dynamics", 2));
  }
}

TEST(TesseractURDFUnit, write_dynamics)  // NOLINT
{
  {
    tesseract_scene_graph::JointDynamics::Ptr dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();
    dynamics->damping = 1.5;
    dynamics->friction = 2.5;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointDynamics::Ptr>(dynamics, &tesseract_urdf::writeDynamics, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract_scene_graph::JointDynamics::Ptr dynamics = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::JointDynamics::Ptr>(dynamics, &tesseract_urdf::writeDynamics, text));
    EXPECT_EQ(text, "");
  }
}
