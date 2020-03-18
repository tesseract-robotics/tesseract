#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/sphere.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_sphere)  // NOLINT
{
  {
    std::string str = R"(<sphere radius="1" extra="0 0 0"/>)";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere", 2);
    EXPECT_TRUE(*status);
    EXPECT_EQ(status->category()->name(), "SphereStatusCategory");
    EXPECT_FALSE(status->category()->message(999).empty());  // Test invalid error code
    EXPECT_FALSE(status->message().empty());
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
  }

  {
    std::string str = R"(<sphere radius="-1" extra="0 0 0"/>)";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  {
    std::string str = R"(<sphere radius="a"/>)";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<sphere radius="1 2"/>)";
  //    tesseract_geometry::Sphere::Ptr geom;
  //    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere", 2);
  //    EXPECT_FALSE(*status);
  //    EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<sphere />)";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere", 2);
    EXPECT_FALSE(*status);
    EXPECT_FALSE(status->message().empty());
  }
}
