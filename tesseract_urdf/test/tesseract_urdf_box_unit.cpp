#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/box.h>
#include <tesseract_geometry/impl/box.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_box)  // NOLINT
{
  {
    std::string str = R"(<box size="1 2.0 3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
    EXPECT_NEAR(geom->getX(), 1, 1e-8);
    EXPECT_NEAR(geom->getY(), 2, 1e-8);
    EXPECT_NEAR(geom->getZ(), 3, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<box size="0.5 0.25 0.75" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
    EXPECT_NEAR(geom->getX(), 0.5, 1e-8);
    EXPECT_NEAR(geom->getY(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getZ(), 0.75, 1e-8);
  }

  {
    std::string str = R"(<box size="-1 2.0 3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }

  {
    std::string str = R"(<box size="1 -2.0 3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }

  {
    std::string str = R"(<box size="1 2.0 -3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }

  {
    std::string str = R"(<box size="1.0 2 a"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }

  {
    std::string str = R"(<box size="1 2"/>)";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }

  {
    std::string str = "<box />";
    tesseract_geometry::Box::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Box::Ptr>(geom, &tesseract_urdf::parseBox, str, "box", 2));
  }
}
