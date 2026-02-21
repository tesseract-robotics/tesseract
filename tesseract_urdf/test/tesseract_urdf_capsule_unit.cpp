#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/capsule.h>
#include <tesseract_geometry/impl/capsule.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_capsule)  // NOLINT
{
  {
    std::string str = R"(<tesseract:capsule radius="1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<tesseract:capsule radius="0.25" length="0.5" extra="0 0 0"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getLength(), 0.5, 1e-8);
  }

  {
    std::string str = R"(<tesseract:capsule radius="-1" length="2" extra="0 0 0"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:capsule radius="1" length="-2" extra="0 0 0"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:capsule radius="a" length="2"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:capsule radius="1" length="a"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<tesseract:capsule radius="1 2" length="2 3"/>)";
  //    tesseract::geometry::Capsule::Ptr geom;
  //    auto status = runTest<tesseract::geometry::Capsule::Ptr>(geom, str,
  //    tesseract_urdf::CAPSULE_ELEMENT_NAME.data()); EXPECT_FALSE(*status); EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<tesseract:capsule radius="1"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<tesseract:capsule length="2"/>)";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<tesseract:capsule />";
    tesseract::geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::Capsule::Ptr>(
        geom, &tesseract::urdf::parseCapsule, str, tesseract::urdf::CAPSULE_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_capsule)  // NOLINT
{
  {
    tesseract::geometry::Capsule::Ptr capsule = std::make_shared<tesseract::geometry::Capsule>(0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract::geometry::Capsule::Ptr>(capsule, &tesseract::urdf::writeCapsule, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract::geometry::Capsule::Ptr capsule = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract::geometry::Capsule::Ptr>(capsule, &tesseract::urdf::writeCapsule, text));
    EXPECT_EQ(text, "");
  }
}
