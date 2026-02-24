#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/urdf/box.h>
#include <tesseract/geometry/impl/box.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_box)  // NOLINT
{
  {
    std::string str = R"(<box size="1 2.0 3" extra="0 0 0"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getX(), 1, 1e-8);
    EXPECT_NEAR(geom->getY(), 2, 1e-8);
    EXPECT_NEAR(geom->getZ(), 3, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<box size="0.5 0.25 0.75" extra="0 0 0"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getX(), 0.5, 1e-8);
    EXPECT_NEAR(geom->getY(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getZ(), 0.75, 1e-8);
  }

  {
    std::string str = R"(<box size="-1 2.0 3" extra="0 0 0"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<box size="1 -2.0 3" extra="0 0 0"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<box size="1 2.0 -3" extra="0 0 0"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<box size="1.0 2 a"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<box size="1 2"/>)";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<box />";
    tesseract::geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Box::Ptr>(
        geom, &tesseract::urdf::parseBox, str, tesseract::urdf::BOX_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_box)  // NOLINT
{
  {
    tesseract::geometry::Box::Ptr geom = std::make_shared<tesseract::geometry::Box>(1.0, 2.0, 3.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::geometry::Box::Ptr>(geom, &tesseract::urdf::writeBox, text));
    EXPECT_EQ(text, R"(<box size="1 2 3"/>)");
  }

  {
    tesseract::geometry::Box::Ptr geom = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::geometry::Box::Ptr>(geom, &tesseract::urdf::writeBox, text));
    EXPECT_EQ(text, "");
  }
}
