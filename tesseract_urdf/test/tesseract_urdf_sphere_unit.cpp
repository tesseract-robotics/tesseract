#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/sphere.h>
#include <tesseract_geometry/impl/sphere.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_sphere)  // NOLINT
{
  {
    std::string str = R"(<sphere radius="1" extra="0 0 0"/>)";
    tesseract::geometry::Sphere::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Sphere::Ptr>(
        geom, &tesseract::urdf::parseSphere, str, tesseract::urdf::SPHERE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<sphere radius="0.25" extra="0 0 0"/>)";
    tesseract::geometry::Sphere::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Sphere::Ptr>(
        geom, &tesseract::urdf::parseSphere, str, tesseract::urdf::SPHERE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
  }

  {
    std::string str = R"(<sphere radius="-1" extra="0 0 0"/>)";
    tesseract::geometry::Sphere::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Sphere::Ptr>(
        geom, &tesseract::urdf::parseSphere, str, tesseract::urdf::SPHERE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<sphere radius="a"/>)";
    tesseract::geometry::Sphere::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Sphere::Ptr>(
        geom, &tesseract::urdf::parseSphere, str, tesseract::urdf::SPHERE_ELEMENT_NAME.data()));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<sphere radius="1 2"/>)";
  //    tesseract::geometry::Sphere::Ptr geom;
  //    auto status = runTest<tesseract::geometry::Sphere::Ptr>(geom, str, tesseract_urdf::SPHERE_ELEMENT_NAME.data());
  //    EXPECT_FALSE(*status);
  //    EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<sphere />)";
    tesseract::geometry::Sphere::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Sphere::Ptr>(
        geom, &tesseract::urdf::parseSphere, str, tesseract::urdf::SPHERE_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_sphere)  // NOLINT
{
  {
    tesseract::geometry::Sphere::Ptr geom = std::make_shared<tesseract::geometry::Sphere>(1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::geometry::Sphere::Ptr>(geom, &tesseract::urdf::writeSphere, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::geometry::Sphere::Ptr geom = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::geometry::Sphere::Ptr>(geom, &tesseract::urdf::writeSphere, text));
    EXPECT_EQ(text, "");
  }
}
