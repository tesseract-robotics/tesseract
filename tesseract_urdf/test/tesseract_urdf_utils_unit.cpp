#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/utils.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, trailingSlash)  // NOLINT
{
  {
    std::string str = "/tmp";
    EXPECT_EQ(tesseract::urdf::trailingSlash(str), "/tmp/");
  }

  {
    std::string str = "/tmp/";
    EXPECT_EQ(tesseract::urdf::trailingSlash(str), "/tmp/");
  }

  {
    std::string str;
    EXPECT_EQ(tesseract::urdf::trailingSlash(str), "/");
  }
}

TEST(TesseractURDFUnit, noTrailingSlash)  // NOLINT
{
  {
    std::string str = "/tmp";
    EXPECT_EQ(tesseract::urdf::noTrailingSlash(str), "/tmp");
  }

  {
    std::string str = "/tmp/";
    EXPECT_EQ(tesseract::urdf::noTrailingSlash(str), "/tmp");
  }

  {
    std::string str;
    EXPECT_EQ(tesseract::urdf::noTrailingSlash(str), "");
  }

  {
    std::string str = "/tmp//";
    EXPECT_EQ(tesseract::urdf::noTrailingSlash(str), "/tmp");
  }
}

TEST(TesseractURDFUnit, noLeadingSlash)  // NOLINT
{
  {
    std::string str = "/tmp";
    EXPECT_EQ(tesseract::urdf::noLeadingSlash(str), "tmp");
  }

  {
    std::string str = "/tmp/";
    EXPECT_EQ(tesseract::urdf::noLeadingSlash(str), "tmp/");
  }

  {
    std::string str = "//tmp/";
    EXPECT_EQ(tesseract::urdf::noLeadingSlash(str), "tmp/");
  }

  {
    std::string str;
    EXPECT_EQ(tesseract::urdf::noLeadingSlash(str), "");
  }
}
