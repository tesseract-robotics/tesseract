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
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_TRUE(*status);
    EXPECT_EQ(status->category()->name(), "DynamicsStatusCategory");
    EXPECT_FALSE(status->category()->message(999).empty());  // Test invalid error code
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="1"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 0, 1e-8);
  }

  {
    std::string str = R"(<dynamics friction="2"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->damping, 0, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = R"(<dynamics damping="a" friction="2"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<dynamics damping="1" friction="b"/>)";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = "<dynamics/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }
}
