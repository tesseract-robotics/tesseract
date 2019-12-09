#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/calibration.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_calibration)  // NOLINT
{
  {
    std::string str = R"(<calibration rising="1" falling="2" extra="0 0 0"/>)";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = R"(<calibration rising="1"/>)";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 0, 1e-8);
  }

  {
    std::string str = R"(<calibration falling="2"/>)";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 0, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = R"(<calibration rising="a" falling="2"/>)";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<calibration rising="1" falling="b"/>)";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<calibration/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration", 2);
    EXPECT_FALSE(*status);
  }
}
