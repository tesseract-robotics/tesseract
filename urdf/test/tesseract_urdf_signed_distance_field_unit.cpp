#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract/common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/signed_distance_field.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/urdf/signed_distance_field.h>
#include <tesseract/common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_signed_distance_field)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  const std::string sdf_url = "package://tesseract/support/meshes/sphere.sdf";

  {  // Filename only -> default scale
    std::string str = R"(<tesseract:signed_distance_field filename=")" + sdf_url + R"(" extra="0 0 0"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
    EXPECT_TRUE(geom != nullptr);
    EXPECT_FALSE(geom->getDistances().empty());
    EXPECT_TRUE(geom->getScale().isOnes());
  }

  {  // Scale set
    std::string str = R"(<tesseract:signed_distance_field filename=")" + sdf_url + R"(" scale="1 2 3"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_TRUE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
    EXPECT_NEAR(geom->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom->getScale()[2], 3, 1e-5);
  }

  {  // Failure: missing filename
    std::string str = R"(<tesseract:signed_distance_field scale="1 2 3"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
  }

  {  // Failure: bad scale token
    std::string str = R"(<tesseract:signed_distance_field filename=")" + sdf_url + R"(" scale="1 a 3"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
  }

  {  // Failure: non-positive scale
    std::string str = R"(<tesseract:signed_distance_field filename=")" + sdf_url + R"(" scale="-1 2 3"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
  }

  {  // Failure: wrong number of scale tokens
    std::string str = R"(<tesseract:signed_distance_field filename=")" + sdf_url + R"(" scale="1 2"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
  }

  {  // Failure: resource does not exist
    std::string str =
        R"(<tesseract:signed_distance_field filename="package://tesseract/support/meshes/does_not_exist.sdf"/>)";
    tesseract::geometry::SignedDistanceField::Ptr geom;
    EXPECT_FALSE(runTest<tesseract::geometry::SignedDistanceField::Ptr>(
        geom,
        &tesseract::urdf::parseSignedDistanceField,
        str,
        tesseract::urdf::SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data(),
        resource_locator));
  }
}

TEST(TesseractURDFUnit, write_signed_distance_field)  // NOLINT
{
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  const auto make_field = [&sphere](const Eigen::Vector3d& scale) {
    return tesseract::geometry::createDiscreteSignedDistanceField(
        sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(4, 4, 4), scale);
  };

  {  // Default scale
    auto sdf = make_field(Eigen::Vector3d(1, 1, 1));
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::geometry::SignedDistanceField::Ptr>(sdf,
                                                                       &tesseract::urdf::writeSignedDistanceField,
                                                                       text,
                                                                       tesseract::common::getTempPath(),
                                                                       std::string("sdf0.sdf")));
    EXPECT_NE(text, "");
  }

  {  // With scale
    auto sdf = make_field(Eigen::Vector3d(0.5, 0.5, 0.5));
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::geometry::SignedDistanceField::Ptr>(sdf,
                                                                       &tesseract::urdf::writeSignedDistanceField,
                                                                       text,
                                                                       tesseract::common::getTempPath(),
                                                                       std::string("sdf1.sdf")));
    EXPECT_NE(text.find("scale=\"0.5 0.5 0.5\""), std::string::npos);
  }

  {  // Failure: unwritable path
    auto sdf = make_field(Eigen::Vector3d(1, 1, 1));
    std::string text;
    EXPECT_EQ(
        1,
        writeTest<tesseract::geometry::SignedDistanceField::Ptr>(
            sdf, &tesseract::urdf::writeSignedDistanceField, text, tesseract::common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {  // Failure: nullptr input
    tesseract::geometry::SignedDistanceField::Ptr sdf = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::geometry::SignedDistanceField::Ptr>(sdf,
                                                                       &tesseract::urdf::writeSignedDistanceField,
                                                                       text,
                                                                       tesseract::common::getTempPath(),
                                                                       std::string("sdf2.sdf")));
    EXPECT_EQ(text, "");
  }
}
