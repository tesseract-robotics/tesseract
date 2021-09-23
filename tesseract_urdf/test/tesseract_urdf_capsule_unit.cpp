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
    std::string str = R"(<capsule radius="1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {  // https://github.com/ros-industrial-consortium/tesseract_ros/issues/67
    std::string str = R"(<capsule radius="0.25" length="0.5" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
    EXPECT_NEAR(geom->getRadius(), 0.25, 1e-8);
    EXPECT_NEAR(geom->getLength(), 0.5, 1e-8);
  }

  {
    std::string str = R"(<capsule radius="-1" length="2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  {
    std::string str = R"(<capsule radius="1" length="-2" extra="0 0 0"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  {
    std::string str = R"(<capsule radius="a" length="2"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  {
    std::string str = R"(<capsule radius="1" length="a"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
  //  {
  //    std::string str = R"(<capsule radius="1 2" length="2 3"/>)";
  //    tesseract_geometry::Capsule::Ptr geom;
  //    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
  //    EXPECT_FALSE(*status);
  //    EXPECT_FALSE(status->message().empty());
  //  }

  {
    std::string str = R"(<capsule radius="1"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  {
    std::string str = R"(<capsule length="2"/>)";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }

  {
    std::string str = "<capsule />";
    tesseract_geometry::Capsule::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Capsule::Ptr>(geom, &tesseract_urdf::parseCapsule, str, "capsule", 2));
  }
}

TEST(TesseractURDFUnit, write_capsule)  // NOLINT
{
  {
    tesseract_geometry::Capsule::Ptr capsule = std::make_shared<tesseract_geometry::Capsule>(0.5, 1.0);
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_geometry::Capsule::Ptr>(capsule, &tesseract_urdf::writeCapsule, text));
    EXPECT_NE(text, "");
  }

  {
    tesseract_geometry::Capsule::Ptr capsule = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_geometry::Capsule::Ptr>(capsule, &tesseract_urdf::writeCapsule, text));
    EXPECT_EQ(text, "");
  }
}
