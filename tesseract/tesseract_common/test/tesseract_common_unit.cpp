#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <type_traits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/sfinae_utils.h>

TEST(TesseractCommonUnit, isNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (const auto& s : true_test)
  {
    EXPECT_TRUE(tesseract_common::isNumeric(s));
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    EXPECT_FALSE(tesseract_common::isNumeric(s));
  }
}

TEST(TesseractCommonUnit, toNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  std::vector<double> true_test_value = { 1, 1.5, -1, -1.5, 1e-5, 1e5, -1e-5, -1e5, 1.0e-5, 1.0e5, -1.0e-5, -1.0e5 };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (size_t i = 0; i < true_test.size(); ++i)
  {
    double value = 0;
    EXPECT_TRUE(tesseract_common::toNumeric<double>(true_test[i], value));
    EXPECT_NEAR(value, true_test_value[i], 1e-8);
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    double value = 0;
    EXPECT_FALSE(tesseract_common::toNumeric(s, value));
    EXPECT_NEAR(value, 0, 1e-8);
  }
}

TEST(TesseractCommonUnit, generateRandomNumber)  // NOLINT
{
  Eigen::MatrixX2d limits(4, 2);
  limits(0, 0) = -5;
  limits(0, 1) = 5;
  limits(1, 0) = 0;
  limits(1, 1) = 10;
  limits(2, 0) = 5;
  limits(2, 1) = 15;
  limits(3, 0) = -15;
  limits(3, 1) = -5;

  Eigen::VectorXd random_numbers = tesseract_common::generateRandomNumber(limits);
  EXPECT_EQ(limits.rows(), random_numbers.size());
  for (long i = 0; i < limits.rows(); ++i)
  {
    EXPECT_LE(random_numbers(i), limits(i, 1));
    EXPECT_GE(random_numbers(i), limits(i, 0));
  }

  Eigen::MatrixX2d empty_limits;
  Eigen::VectorXd random_numbers2 = tesseract_common::generateRandomNumber(empty_limits);
  EXPECT_EQ(empty_limits.rows(), random_numbers2.size());

  Eigen::MatrixX2d equal_limits(4, 2);
  equal_limits(0, 0) = 5;
  equal_limits(0, 1) = 5;
  equal_limits(1, 0) = 5;
  equal_limits(1, 1) = 5;
  equal_limits(2, 0) = 5;
  equal_limits(2, 1) = 5;
  equal_limits(3, 0) = 5;
  equal_limits(3, 1) = 5;
  Eigen::VectorXd random_numbers3 = tesseract_common::generateRandomNumber(equal_limits);
  EXPECT_EQ(equal_limits.rows(), random_numbers3.size());
  for (long i = 0; i < equal_limits.rows(); ++i)
  {
    EXPECT_NEAR(random_numbers3(i), 5, 1e-5);
  }

  Eigen::MatrixX2d wrong_limits(4, 2);
  wrong_limits(0, 0) = 5;
  wrong_limits(0, 1) = -5;
  wrong_limits(1, 0) = 5;
  wrong_limits(1, 1) = -5;
  wrong_limits(2, 0) = 5;
  wrong_limits(2, 1) = -5;
  wrong_limits(3, 0) = 5;
  wrong_limits(3, 1) = -5;
  Eigen::VectorXd random_numbers4 = tesseract_common::generateRandomNumber(wrong_limits);
  EXPECT_EQ(wrong_limits.rows(), random_numbers4.size());
  for (long i = 0; i < limits.rows(); ++i)
  {
    EXPECT_GE(random_numbers4(i), wrong_limits(i, 1));
    EXPECT_LE(random_numbers4(i), wrong_limits(i, 0));
  }
}

TEST(TesseractCommonUnit, trim)  // NOLINT
{
  std::string check1 = "    trim";
  std::string check2 = "trim    ";
  std::string check3 = "    trim    ";
  std::string check_trimmed = "trim";

  std::string s = check1;
  tesseract_common::rtrim(s);
  EXPECT_EQ(s, check1);
  tesseract_common::ltrim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check2;
  tesseract_common::ltrim(s);
  EXPECT_EQ(s, check2);
  tesseract_common::rtrim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check1;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check2;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check3;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);
}

struct TestHasMemberFunction
{
  bool update() const { return true; }
  int add(int a) const { return a + 1; }
};

struct TestHasMemberWithArgFunction
{
  bool update(std::shared_ptr<TestHasMemberWithArgFunction>& p) { return (p == nullptr); }
  double add(double a, double b) const { return a + b; }
};

struct TestMissingMemberFunction
{
  bool missingUpdate() const { return false; }
  double add(int a) const { return a + 1; }
};

CREATE_MEMBER_CHECK(update);
CREATE_MEMBER_FUNC_INVOCABLE_CHECK(update, std::shared_ptr<T>&);
CREATE_MEMBER_FUNC_INVOCABLE_CHECK(add, double, double);
CREATE_MEMBER_FUNC_RETURN_TYPE_CHECK(add, int, int);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(add, double, double, double);

TEST(TesseractCommonUnit, sfinaeHasMemberFunction)  // NOLINT
{
  bool t_true = has_member_update<TestHasMemberFunction>::value;
  bool t_false = has_member_update<TestMissingMemberFunction>::value;
  EXPECT_TRUE(t_true);
  EXPECT_FALSE(t_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionInvocable)  // NOLINT
{
  bool i_update_true = has_member_func_invocable_update<TestHasMemberWithArgFunction>::value;
  bool i_add_true = has_member_func_invocable_add<TestHasMemberWithArgFunction>::value;
  bool i_update_false = has_member_func_invocable_update<TestHasMemberFunction>::value;
  bool i_add_false = has_member_func_invocable_add<TestHasMemberFunction>::value;
  EXPECT_TRUE(i_update_true);
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(i_update_false);
  EXPECT_FALSE(i_add_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionWithReturnType)  // NOLINT
{
  bool i_add_true = has_member_func_return_type_add<TestHasMemberFunction>::value;
  bool t_add_false = has_member_func_return_type_add<TestMissingMemberFunction>::value;
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(t_add_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionSignature)  // NOLINT
{
  bool i_add_true = has_member_func_signature_add<TestHasMemberWithArgFunction>::value;
  bool t_add_false = has_member_func_signature_add<TestMissingMemberFunction>::value;
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(t_add_false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
