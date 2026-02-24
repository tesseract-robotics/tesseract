#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/scene_graph/joint.h>
#include <tesseract/urdf/dynamics.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_dynamics)  // NOLINT
{
  {
    std::string str = R"(<dynamics damping="1" friction="2" extra="0 0 0"/>)";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="1"/>)";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 0, 1e-8);
  }

  {
    std::string str = R"(<dynamics friction="2"/>)";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->damping, 0, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="a" friction="2"/>)";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<dynamics damping="1" friction="b"/>)";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<dynamics/>";
    tesseract::scene_graph::JointDynamics::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointDynamics::Ptr>(
        elem, &tesseract::urdf::parseDynamics, str, tesseract::urdf::DYNAMICS_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_dynamics)  // NOLINT
{
  {
    tesseract::scene_graph::JointDynamics::Ptr dynamics = std::make_shared<tesseract::scene_graph::JointDynamics>();
    dynamics->damping = 1.5;
    dynamics->friction = 2.5;
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::scene_graph::JointDynamics::Ptr>(dynamics, &tesseract::urdf::writeDynamics, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::scene_graph::JointDynamics::Ptr dynamics = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::scene_graph::JointDynamics::Ptr>(dynamics, &tesseract::urdf::writeDynamics, text));
    EXPECT_EQ(text, "");
  }
}
