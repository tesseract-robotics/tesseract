#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/cone.h>
#include <tesseract_geometry/impl/cone.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_cone)  // NOLINT
{
  {
    std::string str = R"(<tesseract:cone radius="1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<tesseract:cone radius="0.25" length="0.5" extra="0 0 0"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getLength(), 0.5, 1e-8);
  }

  {
    std::string str = R"(<tesseract:cone radius="-1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:cone radius="1" length="-2" extra="0 0 0"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:cone radius="a" length="2"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:cone radius="1" length="a"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<tesseract:cone radius="1 2" length="2 3"/>)";
  //    tesseract::geometry::Cone::Ptr geom;
  //    auto status = runTest<tesseract::geometry::Cone::Ptr>(geom, str, tesseract_urdf::CONE_ELEMENT_NAME.data());
  //    EXPECT_FALSE(*status);
  //    EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<tesseract:cone radius="1"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:cone length="2"/>)";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<tesseract:cone />";
    tesseract::geometry::Cone::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Cone::Ptr>(
        geom, &tesseract::urdf::parseCone, str, tesseract::urdf::CONE_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_cone)  // NOLINT
{
  {
    tesseract::geometry::Cone::Ptr cone = std::make_shared<tesseract::geometry::Cone>(0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::geometry::Cone::Ptr>(cone, &tesseract::urdf::writeCone, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::geometry::Cone::Ptr cone = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::geometry::Cone::Ptr>(cone, &tesseract::urdf::writeCone, text));
    EXPECT_EQ(text, "");
  }
}
