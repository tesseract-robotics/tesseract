#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/capsule.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_capsule)  // NOLINT
{
  {
    std::string str = R"(<capsule radius="1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {
    std::string str = R"(<capsule radius="-1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<capsule radius="1" length="-2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<capsule radius="a" length="2"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<capsule radius="1" length="a"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<capsule radius="1 2" length="2 3"/>)";
  //    tesseract_geometry::Capsule::Ptr geom;
  //    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
  //    EXPECT_FALSE(*status);
  //  }

  {
    std::string str = R"(<capsule radius="1"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<capsule length="2"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<capsule />";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule", 2);
    EXPECT_FALSE(*status);
  }
}
