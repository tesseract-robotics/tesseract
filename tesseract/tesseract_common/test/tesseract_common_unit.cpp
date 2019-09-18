#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

TEST(TesseractCommonUnit, isNumeric)
{
  std::vector<std::string> true_test = {"1", "1.5", "-1", "-1.5", "1e-5", "1e5", "-1e-5","-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5"};

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (const auto& s : true_test)
  {
    EXPECT_TRUE(tesseract_common::isNumeric(s));
  }

  std::vector<std::string> false_test = {"a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "="};
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    EXPECT_FALSE(tesseract_common::isNumeric(s));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
