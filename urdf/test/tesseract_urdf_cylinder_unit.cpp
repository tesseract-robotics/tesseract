#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/urdf/cylinder.h>
#include <tesseract/geometry/impl/cylinder.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_cylinder)  // NOLINT
{
  {
    std::string str = R"(<cylinder radius="1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<cylinder radius="0.25" length="0.5" extra="0 0 0"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getLength(), 0.5, 1e-8);
  }

  {
    std::string str = R"(<cylinder radius="-1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<cylinder radius="1" length="-2" extra="0 0 0"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<cylinder radius="a" length="2"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<cylinder radius="1" length="a"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<cylinder radius="1 2" length="2 3"/>)";
  //    tesseract::geometry::Cylinder::Ptr geom;
  //    auto status = runTest<tesseract::geometry::Cylinder::Ptr>(geom, str,
  //    tesseract_urdf::CYLINDER_ELEMENT_NAME.data()); EXPECT_FALSE(*status); EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<cylinder radius="1"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<cylinder length="2"/>)";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<cylinder />";
    tesseract::geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cylinder::Ptr>(
        geom, &tesseract::urdf::parseCylinder, str, tesseract::urdf::CYLINDER_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_cylinder)  // NOLINT
{
  {
    tesseract::geometry::Cylinder::Ptr cylinder = std::make_shared<tesseract::geometry::Cylinder>(0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::geometry::Cylinder::Ptr>(cylinder, &tesseract::urdf::writeCylinder, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::geometry::Cylinder::Ptr cylinder = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::geometry::Cylinder::Ptr>(cylinder, &tesseract::urdf::writeCylinder, text));
    EXPECT_EQ(text, "");
  }
}
