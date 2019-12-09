#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/cylinder.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_cylinder)  // NOLINT
{
  {
    std::string str = R"(<cylinder radius="1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {
    std::string str = R"(<cylinder radius="-1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<cylinder radius="1" length="-2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<cylinder radius="a" length="2"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<cylinder radius="1" length="a"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<cylinder radius="1 2" length="2 3"/>)";
  //    tesseract_geometry::Cylinder::Ptr geom;
  //    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
  //    EXPECT_FALSE(*status);
  //  }

  {
    std::string str = R"(<cylinder radius="1"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<cylinder length="2"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cylinder />";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
    EXPECT_FALSE(*status);
  }
}
