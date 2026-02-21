#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/calibration.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_calibration)  // NOLINT
{
  {
    std::string str = R"(<calibration rising="1" falling="2" extra="0 0 0"/>)";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = R"(<calibration rising="1"/>)";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 0, 1e-8);
  }

  {
    std::string str = R"(<calibration falling="2"/>)";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->rising, 0, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = R"(<calibration rising="a" falling="2"/>)";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<calibration rising="1" falling="b"/>)";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<calibration/>";
    tesseract::scene_graph::JointCalibration::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::JointCalibration::Ptr>(
        elem, &tesseract::urdf::parseCalibration, str, tesseract::urdf::CALIBRATION_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_calibration)  // NOLINT
{
  {
    tesseract::scene_graph::JointCalibration::Ptr cal = std::make_shared<tesseract::scene_graph::JointCalibration>();
    cal->rising = 5.0;
    cal->falling = 3.0;
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::scene_graph::JointCalibration::Ptr>(cal, &tesseract::urdf::writeCalibration, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::scene_graph::JointCalibration::Ptr cal = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::scene_graph::JointCalibration::Ptr>(cal, &tesseract::urdf::writeCalibration, text));
    EXPECT_EQ(text, "");
  }
}
