#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract_geometry/impl/box.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/box.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_box)  // NOLINT
{
  {
    std::string str = R"(<box size="1 2.0 3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box", 2);
    EXPECT_TRUE(*status);

    EXPECT_NEAR(geom->getX(), 1, 1e-8);
    EXPECT_NEAR(geom->getY(), 2, 1e-8);
    EXPECT_NEAR(geom->getZ(), 3, 1e-8);
  }

  {
    std::string str = R"(<box size="-1 2.0 3" extra="0 0 0"/>)";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<box size="1.0 2 a"/>)";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<box size="1 2"/>)";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<box />";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box", 2);
    EXPECT_FALSE(*status);
  }
}
