#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/safety_controller.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_safety_controller)  // NOLINT
{
  {
    std::string str =
        R"(<safety_controller soft_lower_limit="1" soft_upper_limit="2" k_position="3" k_velocity="4" extra="0 0 0"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_TRUE(*status);
    EXPECT_EQ(status->category()->name(), "SafetyStatusCategory");
    EXPECT_FALSE(status->category()->message(999).empty());  // Test invalid error code
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<safety_controller soft_upper_limit="2" k_position="3" k_velocity="4"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->soft_lower_limit, 0, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<safety_controller soft_lower_limit="1" k_position="3" k_velocity="4"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 0, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<safety_controller soft_lower_limit="1" soft_upper_limit="2" k_velocity="4"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 0, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<safety_controller k_velocity="4"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_TRUE(*status);
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(elem->soft_lower_limit, 0, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 0, 1e-8);
    EXPECT_NEAR(elem->k_position, 0, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<safety_controller soft_lower_limit="1" soft_upper_limit="2" k_position="3"/>)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<safety_controller />)";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }
}
