#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}
