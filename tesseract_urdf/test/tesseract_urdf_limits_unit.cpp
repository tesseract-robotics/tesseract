#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/limits.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_limits)  // NOLINT
{
  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="4" extra="0 0 0"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_TRUE(*status);
    EXPECT_EQ(status->category()->name(), "LimitsStatusCategory");
    EXPECT_FALSE(status->category()->message(999).empty());  // Test invalid error code
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit upper="2" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit lower="1" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit lower="a" upper="2" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<limit lower="1" upper="a" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="a" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="a"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<limit velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<limit effort="3"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = "<limit />";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }
}
