#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

TEST(TesseractROSUtilsUnit, Instantiation)  // NOLINT
{
  using namespace tesseract_rosutils;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
