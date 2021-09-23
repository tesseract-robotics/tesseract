#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/cylinder.h>
#include <tesseract_geometry/impl/cylinder.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_cylinder)  // NOLINT
{
  {
    std::string str = R"(<cylinder radius="1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<cylinder radius="0.25" length="0.5" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getLength(), 0.5, 1e-8);
  }

  {
    std::string str = R"(<cylinder radius="-1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  {
    std::string str = R"(<cylinder radius="1" length="-2" extra="0 0 0"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  {
    std::string str = R"(<cylinder radius="a" length="2"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  {
    std::string str = R"(<cylinder radius="1" length="a"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<cylinder radius="1 2" length="2 3"/>)";
  //    tesseract_geometry::Cylinder::Ptr geom;
  //    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder", 2);
  //    EXPECT_FALSE(*status);
  //    EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<cylinder radius="1"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  {
    std::string str = R"(<cylinder length="2"/>)";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }

  {
    std::string str = "<cylinder />";
    tesseract_geometry::Cylinder::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Cylinder::Ptr>(geom, &tesseract_urdf::parseCylinder, str, "cylinder", 2));
  }
}

TEST(TesseractURDFUnit, write_cylinder)  // NOLINT
{
  {
    tesseract_geometry::Cylinder::Ptr cylinder = std::make_shared<tesseract_geometry::Cylinder>(0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_geometry::Cylinder::Ptr>(cylinder, &tesseract_urdf::writeCylinder, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract_geometry::Cylinder::Ptr cylinder = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_geometry::Cylinder::Ptr>(cylinder, &tesseract_urdf::writeCylinder, text));
    EXPECT_EQ(text, "");
  }
}
