#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <type_traits>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/sfinae_utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/types.h>
#include <tesseract_common/any.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/yaml_utils.h>

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

  std::string empty_string;
  EXPECT_FALSE(tesseract_common::isNumeric(empty_string));
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

  std::string empty_string;
  double value = 0;
  EXPECT_FALSE(tesseract_common::toNumeric(empty_string, value));
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
  EXPECT_EQ(limits.rows(), random_numbers.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    EXPECT_LE(random_numbers(i), limits(i, 1));
    EXPECT_GE(random_numbers(i), limits(i, 0));
  }

  Eigen::MatrixX2d empty_limits;
  Eigen::VectorXd random_numbers2 = tesseract_common::generateRandomNumber(empty_limits);
  EXPECT_EQ(empty_limits.rows(), random_numbers2.rows());

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
  EXPECT_EQ(equal_limits.rows(), random_numbers3.rows());
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
  EXPECT_EQ(wrong_limits.rows(), random_numbers4.rows());
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
  bool update() const { return true; }    // NOLINT
  int add(int a) const { return a + 1; }  // NOLINT
};

struct TestHasMemberWithArgFunction
{
  bool update(std::shared_ptr<TestHasMemberWithArgFunction>& p) { return (p == nullptr); }  // NOLINT
  double add(double a, double b) const { return a + b; }                                    // NOLINT
};

struct TestMissingMemberFunction
{
  bool missingUpdate() const { return false; }  // NOLINT
  double add(int a) const { return a + 1; }     // NOLINT
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

TEST(TesseractCommonUnit, bytesResource)  // NOLINT
{
  std::vector<uint8_t> data;
  for (uint8_t i = 0; i < 8; i++)
  {
    data.push_back(i);
  }

  std::shared_ptr<tesseract_common::BytesResource> bytes_resource =
      std::make_shared<tesseract_common::BytesResource>("package://test_package/data.bin", data);
  EXPECT_EQ(bytes_resource->getUrl(), "package://test_package/data.bin");
  EXPECT_EQ(bytes_resource->isFile(), false);
  EXPECT_EQ(bytes_resource->getFilePath(), "");
  EXPECT_EQ(bytes_resource->locateResource("test"), nullptr);
  auto data2 = bytes_resource->getResourceContents();
  ASSERT_EQ(data.size(), data2.size());
  for (size_t i = 0; i < data.size(); i++)
  {
    EXPECT_EQ(data[i], data2[i]);
  }
  auto data2_stream = bytes_resource->getResourceContentStream();
  for (unsigned char& i : data)
  {
    char data2_val{ 0 };
    data2_stream->read(&data2_val, 1);
    EXPECT_EQ(i, *reinterpret_cast<uint8_t*>(&data2_val));  // NOLINT
  }

  std::shared_ptr<tesseract_common::BytesResource> bytes_resource2 =
      std::make_shared<tesseract_common::BytesResource>("package://test_package/data.bin", &data[0], data.size());
  EXPECT_EQ(bytes_resource2->getUrl(), "package://test_package/data.bin");
  EXPECT_EQ(bytes_resource->getResourceContents().size(), data.size());
}

TEST(TesseractCommonUnit, ManipulatorInfo)  // NOLINT
{
  // Empty tcp
  tesseract_common::ManipulatorInfo manip_info;
  EXPECT_TRUE(manip_info.empty());
  EXPECT_TRUE(manip_info.tcp_frame.empty());
  EXPECT_TRUE(manip_info.manipulator.empty());
  EXPECT_TRUE(manip_info.manipulator_ik_solver.empty());
  EXPECT_TRUE(manip_info.working_frame.empty());

  tesseract_common::ManipulatorInfo manip_info_override("manipulator", "world", "tool0");
  manip_info_override.tcp_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.0, 0.0, 0.25);
  manip_info_override.manipulator_ik_solver = "OPWInvKin";
  manip_info_override.working_frame = "base_link";

  manip_info = manip_info.getCombined(manip_info_override);
  EXPECT_FALSE(manip_info.empty());
  EXPECT_TRUE(manip_info.tcp_frame == manip_info_override.tcp_frame);
  EXPECT_EQ(manip_info.manipulator, manip_info_override.manipulator);
  EXPECT_EQ(manip_info.manipulator_ik_solver, manip_info_override.manipulator_ik_solver);
  EXPECT_EQ(manip_info.working_frame, manip_info_override.working_frame);

  // Test empty method
  {
    tesseract_common::ManipulatorInfo manip_info("manip", "world", "");
    EXPECT_TRUE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info("manip", "", "tool0");
    EXPECT_TRUE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info("", "world", "tool0");
    EXPECT_TRUE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info("", "", "");
    manip_info.manipulator_ik_solver = "manip";
    EXPECT_TRUE(manip_info.empty());
  }
}

TEST(TesseractCommonUnit, JointStateTest)  // NOLINT
{
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3" };
  Eigen::VectorXd positons = Eigen::VectorXd::Constant(3, 5);
  tesseract_common::JointState joint_state(joint_names, positons);
  EXPECT_TRUE(joint_state.joint_names == joint_names);
  EXPECT_TRUE(joint_state.position.isApprox(positons, 1e-5));
}

TESSERACT_ANY_EXPORT(tesseract_common, JointState);  // NOLINT

TEST(TesseractCommonUnit, anyUnit)  // NOLINT
{
  tesseract_common::Any any_type;
  EXPECT_TRUE(any_type.getType() == std::type_index(typeid(nullptr)));

  tesseract_common::JointState joint_state;
  joint_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  joint_state.position = Eigen::VectorXd::Constant(3, 5);
  joint_state.velocity = Eigen::VectorXd::Constant(3, 6);
  joint_state.acceleration = Eigen::VectorXd::Constant(3, 7);
  joint_state.effort = Eigen::VectorXd::Constant(3, 8);
  joint_state.time = 100;

  any_type = joint_state;
  EXPECT_TRUE(any_type.getType() == std::type_index(typeid(tesseract_common::JointState)));
  EXPECT_TRUE(any_type.as<tesseract_common::JointState>() == joint_state);

  // Check clone
  tesseract_common::Any any_copy = any_type;
  EXPECT_TRUE(any_copy == any_type);

  // Check to make sure it is not making a copy during cast
  auto& any_type_ref1 = any_type.as<tesseract_common::JointState>();
  auto& any_type_ref2 = any_type.as<tesseract_common::JointState>();
  auto& any_copy_ref = any_copy.as<tesseract_common::JointState>();
  EXPECT_TRUE(&any_type_ref1 == &any_type_ref2);
  EXPECT_TRUE(&any_type_ref1 != &any_copy_ref);
  EXPECT_TRUE(&any_type_ref2 != &any_copy_ref);

  const auto& any_type_const_ref1 = any_type.as<tesseract_common::JointState>();
  const auto& any_type_const_ref2 = any_type.as<tesseract_common::JointState>();
  EXPECT_TRUE(&any_type_const_ref1 == &any_type_const_ref2);

  {
    std::ofstream os(tesseract_common::getTempPath() + "any_type_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(any_type);
  }

  tesseract_common::Any nany_type;
  {
    std::ifstream ifs(tesseract_common::getTempPath() + "any_type_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(nany_type);
  }

  EXPECT_TRUE(nany_type.getType() == std::type_index(typeid(tesseract_common::JointState)));
  EXPECT_TRUE(nany_type.as<tesseract_common::JointState>() == joint_state);

  // Test bad cast
  EXPECT_ANY_THROW(nany_type.as<tesseract_common::Toolpath>());  // NOLINT
}

TEST(TesseractCommonUnit, boundsUnit)  // NOLINT
{
  Eigen::VectorXd v = Eigen::VectorXd::Ones(6);
  v = v.array() + std::numeric_limits<float>::epsilon();
  Eigen::MatrixX2d limits(6, 2);
  limits.col(0) = -Eigen::VectorXd::Ones(6);
  limits.col(1) = Eigen::VectorXd::Ones(6);

  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<float>::epsilon()));
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));

  v = -Eigen::VectorXd::Ones(6);
  v = v.array() - std::numeric_limits<float>::epsilon();

  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<float>::epsilon()));
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));

  // Check that clamp is done correctly on both sides
  v = Eigen::VectorXd::Constant(6, -2);
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  ASSERT_EQ((v - limits.col(0)).norm(), 0);

  v = Eigen::VectorXd::Constant(6, 2);
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  ASSERT_EQ((v - limits.col(1)).norm(), 0);
}

TEST(TesseractCommonUnit, isIdenticalUnit)  // NOLINT
{
  std::vector<std::string> v1{ "a", "b", "c" };
  std::vector<std::string> v2{ "a", "b", "c" };
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, true));

  v2 = { "c", "b", "a" };
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, true));

  v2 = { "a", "b", "d" };
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, true));
}

TEST(TesseractCommonUnit, isIdenticalMapUnit)  // NOLINT
{
  std::map<std::string, int> v1;
  v1["1"] = 1;
  v1["2"] = 2;
  std::map<std::string, int> v2;
  bool equal = tesseract_common::isIdenticalMap<std::map<std::string, int>, int>(v1, v2);
  EXPECT_FALSE(equal);

  v2["2"] = 2;
  equal = tesseract_common::isIdenticalMap<std::map<std::string, int>, int>(v1, v2);
  EXPECT_FALSE(equal);

  v2 = v1;
  equal = tesseract_common::isIdenticalMap<std::map<std::string, int>, int>(v1, v2);
  EXPECT_TRUE(equal);

  v1.clear();
  equal = tesseract_common::isIdenticalMap<std::map<std::string, int>, int>(v1, v2);
  EXPECT_FALSE(equal);
}

TEST(TesseractCommonUnit, isIdenticalSetUnit)  // NOLINT
{
  std::set<int> v1;
  std::set<int> v2;
  bool equal = tesseract_common::isIdenticalSet<int>(v1, v2);
  EXPECT_TRUE(equal);

  v1.insert(1);
  equal = tesseract_common::isIdenticalSet<int>(v1, v2);
  EXPECT_FALSE(equal);

  v2.insert(1);
  v2.insert(2);
  equal = tesseract_common::isIdenticalSet<int>(v1, v2);
  EXPECT_FALSE(equal);

  v1.insert(2);
  equal = tesseract_common::isIdenticalSet<int>(v1, v2);
  EXPECT_TRUE(equal);
}

TEST(TesseractCommonUnit, isIdenticalArrayUnit)  // NOLINT
{
  {
    std::array<int, 4> v1 = { 1, 2, 3, 4 };
    std::array<int, 4> v2 = { 1, 2, 3, 4 };
    bool equal = tesseract_common::isIdenticalArray<int, 4>(v1, v2);
    EXPECT_TRUE(equal);
  }
  {
    std::array<int, 4> v1 = { 1, 2, 3, 4 };
    std::array<int, 4> v2 = { -1, 2, 3, 4 };
    bool equal = tesseract_common::isIdenticalArray<int, 4>(v1, v2);
    EXPECT_FALSE(equal);
  }
  {
    // Clang-tidy catches unitialized arrays anyway, but check it just in case the caller isn't running clang-tidy
    std::array<int, 4> v1 = { 1, 2, 3, 6 };
    std::array<int, 4> v2;  // NOLINT
    bool equal = tesseract_common::isIdenticalArray<int, 4>(v1, v2);
    EXPECT_FALSE(equal);
  }
}

TEST(TesseractCommonUnit, pointersEqual)  // NOLINT
{
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = std::make_shared<int>(2);
    bool equal = tesseract_common::pointersEqual(p1, p2);
    EXPECT_FALSE(equal);
  }
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = nullptr;
    bool equal = tesseract_common::pointersEqual<int>(p1, p2);
    EXPECT_FALSE(equal);
  }
  {
    auto p1 = nullptr;
    auto p2 = std::make_shared<int>(2);
    bool equal = tesseract_common::pointersEqual<int>(p1, p2);
    EXPECT_FALSE(equal);
  }
  {
    auto p1 = nullptr;
    auto p2 = nullptr;
    bool equal = tesseract_common::pointersEqual<int>(p1, p2);
    EXPECT_TRUE(equal);
  }
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = std::make_shared<int>(1);
    bool equal = tesseract_common::pointersEqual<int>(p1, p2);
    EXPECT_TRUE(equal);
  }
}

TEST(TesseractCommonUnit, pointersComparison)  // NOLINT
{
  // True if p1 < p2
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = std::make_shared<int>(2);
    bool equal = tesseract_common::pointersComparison<int>(p1, p2);
    EXPECT_TRUE(equal);
  }
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = nullptr;
    bool equal = tesseract_common::pointersComparison<int>(p1, p2);
    EXPECT_FALSE(equal);
  }
  {
    auto p1 = nullptr;
    auto p2 = std::make_shared<int>(2);
    bool equal = tesseract_common::pointersComparison<int>(p1, p2);
    EXPECT_TRUE(equal);
  }
  {
    auto p1 = nullptr;
    auto p2 = nullptr;
    bool equal = tesseract_common::pointersComparison<int>(p1, p2);
    EXPECT_FALSE(equal);
  }
  {
    auto p1 = std::make_shared<int>(1);
    auto p2 = std::make_shared<int>(1);
    bool equal = tesseract_common::pointersComparison<int>(p1, p2);
    EXPECT_FALSE(equal);
  }
}

TEST(TesseractCommonUnit, getTimestampStringUnit)  // NOLINT
{
  std::string s1 = tesseract_common::getTimestampString();
  EXPECT_FALSE(s1.empty());
}

TEST(TesseractCommonUnit, reorder)  // NOLINT
{
  std::vector<std::vector<Eigen::Index>> checks;
  checks.push_back({ 5, 4, 3, 2, 1, 0 });
  checks.push_back({ 0, 1, 2, 3, 4, 5 });
  checks.push_back({ 3, 2, 4, 1, 5, 0 });
  Eigen::VectorXd v = Eigen::VectorXd::Random(6);

  for (const auto& check : checks)
  {
    Eigen::VectorXd v_copy = v;
    tesseract_common::reorder(v_copy, check);

    for (std::size_t i = 0; i < check.size(); ++i)
    {
      EXPECT_NEAR(v_copy(static_cast<Eigen::Index>(i)), v(check[i]), 1e-8);
    }
  }
}

TEST(TesseractCommonUnit, getTempPathUnit)  // NOLINT
{
  std::string s1 = tesseract_common::getTempPath();
  EXPECT_FALSE(s1.empty());
  EXPECT_TRUE(tesseract_common::fs::exists(s1));
}

TEST(TesseractCommonUnit, QueryStringValueUnit)  // NOLINT
{
  {
    std::string str = R"(<box>Test</box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringValue(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "box");
  }
}

TEST(TesseractCommonUnit, QueryStringTextUnit)  // NOLINT
{
  {
    std::string str = R"(<box>Test</box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "Test");
  }

  {
    std::string str = R"(<box></box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, QueryStringAttributeUnit)  // NOLINT
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box missing="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, StringAttributeUnit)  // NOLINT
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value = tesseract_common::StringAttribute(element, "name", "default");
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value = tesseract_common::StringAttribute(element, "missing", "default");
    EXPECT_TRUE(string_value == "default");
  }
}

TEST(TesseractCommonUnit, QueryStringAttributeRequiredUnit)  // NOLINT
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(element, "missing", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, QueryDoubleAttributeRequiredUnit)  // NOLINT
{
  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "name", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_NEAR(double_value, 1.5, 1e-6);
  }

  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "missing", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }

  {
    std::string str = R"(<box name="abc" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "name", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE);
  }
}

TEST(TesseractCommonUnit, QueryIntAttributeRequiredUnit)  // NOLINT
{
  {
    std::string str = R"(<box name="1" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "name", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_NEAR(int_value, 1, 1e-6);
  }

  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "missing", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }

  {
    std::string str = R"(<box name="abc" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "name", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE);
  }
}

// sample function that catches an exception and wraps it in a nested exception
void runThrowNestedException()
{
  try
  {
    throw std::runtime_error("failed");
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("runThrowNestedException() failed"));
  }
}

TEST(TesseractCommonUnit, printNestedExceptionUnit)  // NOLINT
{
  try
  {
    runThrowNestedException();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
  }
}

TEST(TesseractCommonUnit, almostEqualRelativeAndAbsUnit)  // NOLINT
{
  double a = 1e-5;
  double b = 0;
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 1e-7;
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 100000000000000.01;
  b = 100000000000000;
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 100000000000000.1;
  b = 100000000000000;
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  Eigen::VectorXd v1 = Eigen::VectorXd::Constant(3, 1e-5);
  Eigen::VectorXd v2 = Eigen::VectorXd::Constant(3, 0);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 1e-7);
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 100000000000000.01);
  v2 = Eigen::VectorXd::Constant(3, 100000000000000);
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 100000000000000.1);
  v2 = Eigen::VectorXd::Constant(3, 100000000000000);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v2 = Eigen::VectorXd::Constant(1, 100000000000000);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(Eigen::VectorXd(), Eigen::VectorXd()));
}

TEST(TesseractCommonUnit, kinematicsPluginInfoUnit)  // NOLINT
{
  tesseract_common::KinematicsPluginInfo kpi;
  EXPECT_TRUE(kpi.empty());

  tesseract_common::KinematicsPluginInfo kpi_insert;
  kpi_insert.search_paths.insert("/usr/local/lib");
  kpi_insert.search_libraries.insert("tesseract_collision");

  {
    tesseract_common::PluginInfo pi;
    pi.class_name = "KDLFwdKin";
    kpi.fwd_plugin_infos["manipulator"].plugins = { std::make_pair("KDLFwdKin", pi) };
  }

  {
    tesseract_common::PluginInfo pi;
    pi.class_name = "KDLInvKin";
    kpi.inv_plugin_infos["manipulator"].plugins = { std::make_pair("KDLInvKin", pi) };
  }

  EXPECT_FALSE(kpi_insert.empty());

  kpi.insert(kpi_insert);
  EXPECT_FALSE(kpi.empty());

  kpi.clear();
  EXPECT_TRUE(kpi.empty());
}

TEST(TesseractCommonUnit, ContactManagersPluginInfoUnit)  // NOLINT
{
  tesseract_common::ContactManagersPluginInfo cmpi;
  EXPECT_TRUE(cmpi.empty());

  tesseract_common::ContactManagersPluginInfo cmpi_insert;
  cmpi_insert.search_paths.insert("/usr/local/lib");
  cmpi_insert.search_libraries.insert("tesseract_collision");

  {
    tesseract_common::PluginInfo pi;
    pi.class_name = "DiscretePluginFactory";
    cmpi.discrete_plugin_infos.plugins = { std::make_pair("DiscretePlugin", pi) };
  }

  {
    tesseract_common::PluginInfo pi;
    pi.class_name = "ContinuousPluginFactory";
    cmpi.continuous_plugin_infos.plugins = { std::make_pair("ContinuousPlugin", pi) };
  }

  EXPECT_FALSE(cmpi_insert.empty());

  cmpi.insert(cmpi_insert);
  EXPECT_FALSE(cmpi.empty());

  cmpi.clear();
  EXPECT_TRUE(cmpi.empty());
}

TEST(TesseractContactManagersFactoryUnit, KinematicsPluginInfoYamlUnit)  // NOLINT
{
  std::string yaml_string = R"(kinematic_plugins:
                                 search_paths:
                                   - /usr/local/lib
                                 search_libraries:
                                   - tesseract_kinematics_kdl_factories
                                 fwd_kin_plugins:
                                   iiwa_manipulator:
                                     default: KDLFwdKinChain
                                     plugins:
                                       KDLFwdKinChain:
                                         class: KDLFwdKinChainFactory
                                         config:
                                           base_link: base_link
                                           tip_link: tool0
                                 inv_kin_plugins:
                                   iiwa_manipulator:
                                     default: KDLInvKinChainLMA
                                     plugins:
                                       KDLInvKinChainLMA:
                                         class: KDLInvKinChainLMAFactory
                                         config:
                                           base_link: base_link
                                           tip_link: tool0
                                       KDLInvKinChainNR:
                                         class: KDLInvKinChainNRFactory
                                         config:
                                           base_link: base_link
                                           tip_link: tool0)";

  {  // Success
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    auto cmpi = config.as<tesseract_common::KinematicsPluginInfo>();

    const YAML::Node& plugin_info = plugin_config["kinematic_plugins"];
    const YAML::Node& search_paths = plugin_info["search_paths"];
    const YAML::Node& search_libraries = plugin_info["search_libraries"];
    const YAML::Node& fwd_kin_default_plugin = plugin_info["fwd_kin_plugins"]["iiwa_manipulator"]["default"];
    const YAML::Node& fwd_kin_plugins = plugin_info["fwd_kin_plugins"]["iiwa_manipulator"]["plugins"];
    const YAML::Node& inv_kin_default_plugin = plugin_info["inv_kin_plugins"]["iiwa_manipulator"]["default"];
    const YAML::Node& inv_kin_plugins = plugin_info["inv_kin_plugins"]["iiwa_manipulator"]["plugins"];

    {
      std::set<std::string> sp = cmpi.search_paths;
      EXPECT_EQ(sp.size(), 1);

      for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
      {
        EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
      }
    }

    {
      std::set<std::string> sl = cmpi.search_libraries;
      EXPECT_EQ(sl.size(), 1);

      for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
      {
        EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
      }
    }

    EXPECT_EQ(fwd_kin_default_plugin.as<std::string>(), cmpi.fwd_plugin_infos["iiwa_manipulator"].default_plugin);
    EXPECT_EQ(fwd_kin_plugins.size(), cmpi.fwd_plugin_infos["iiwa_manipulator"].plugins.size());

    EXPECT_EQ(inv_kin_default_plugin.as<std::string>(), cmpi.inv_plugin_infos["iiwa_manipulator"].default_plugin);
    EXPECT_EQ(inv_kin_plugins.size(), cmpi.inv_plugin_infos["iiwa_manipulator"].plugins.size());
  }

  {  // search_paths failure
    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     failure: issue
                                   search_libraries:
                                     - tesseract_kinematics_kdl_factories
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLFwdKinChain
                                       plugins:
                                         KDLFwdKinChain:
                                           class: KDLFwdKinChainFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLInvKinChainLMA
                                       plugins:
                                         KDLInvKinChainLMA:
                                           class: KDLInvKinChainLMAFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                         KDLInvKinChainNR:
                                           class: KDLInvKinChainNRFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }

  {  // search_libraries failure
    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     failure: issue
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLFwdKinChain
                                       plugins:
                                         KDLFwdKinChain:
                                           class: KDLFwdKinChainFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLInvKinChainLMA
                                       plugins:
                                         KDLInvKinChainLMA:
                                           class: KDLInvKinChainLMAFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                         KDLInvKinChainNR:
                                           class: KDLInvKinChainNRFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }

  {  // missing fwd plugins failure
    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_kinematics_kdl_factories
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLFwdKinChain
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLInvKinChainLMA
                                       plugins:
                                         KDLInvKinChainLMA:
                                           class: KDLInvKinChainLMAFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                         KDLInvKinChainNR:
                                           class: KDLInvKinChainNRFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }

  {  // fwd plugins is not map failure

    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_kinematics_kdl_factories
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       - tesseract_collision_bullet_factories
                                       - tesseract_collision_fcl_factories
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLInvKinChainLMA
                                       plugins:
                                         KDLInvKinChainLMA:
                                           class: KDLInvKinChainLMAFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                         KDLInvKinChainNR:
                                           class: KDLInvKinChainNRFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }

  {  // missing inv plugins failure
    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_kinematics_kdl_factories
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLFwdKinChain
                                       plugins:
                                         KDLFwdKinChain:
                                           class: KDLFwdKinChainFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLInvKinChainLMA)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }

  {  // inv plugins is not map failure
    std::string yaml_string = R"(kinematic_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_kinematics_kdl_factories
                                   fwd_kin_plugins:
                                     iiwa_manipulator:
                                       default: KDLFwdKinChain
                                       plugins:
                                         KDLFwdKinChain:
                                           class: KDLFwdKinChainFactory
                                           config:
                                             base_link: base_link
                                             tip_link: tool0
                                   inv_kin_plugins:
                                     iiwa_manipulator:
                                       - tesseract_collision_bullet_factories
                                       - tesseract_collision_fcl_factories)";

    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::KinematicsPluginInfo>());  // NOLINT
  }
}

TEST(TesseractContactManagersFactoryUnit, ContactManagersPluginInfoYamlUnit)  // NOLINT
{
  std::string yaml_string = R"(contact_manager_plugins:
                                 search_paths:
                                   - /usr/local/lib
                                 search_libraries:
                                   - tesseract_collision_bullet_factories
                                   - tesseract_collision_fcl_factories
                                 discrete_plugins:
                                   default: BulletDiscreteBVHManager
                                   plugins:
                                     BulletDiscreteBVHManager:
                                       class: BulletDiscreteBVHManagerFactory
                                     BulletDiscreteSimpleManager:
                                       class: BulletDiscreteSimpleManagerFactory
                                     FCLDiscreteBVHManager:
                                       class: FCLDiscreteBVHManagerFactory
                                 continuous_plugins:
                                   default: BulletCastBVHManager
                                   plugins:
                                     BulletCastBVHManager:
                                       class: BulletCastBVHManagerFactory
                                     BulletCastSimpleManager:
                                       class: BulletCastSimpleManagerFactory)";

  {  // Success
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    auto cmpi = config.as<tesseract_common::ContactManagersPluginInfo>();

    const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
    const YAML::Node& search_paths = plugin_info["search_paths"];
    const YAML::Node& search_libraries = plugin_info["search_libraries"];
    const YAML::Node& discrete_default_plugin = plugin_info["discrete_plugins"]["default"];
    const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];
    const YAML::Node& continuous_default_plugin = plugin_info["continuous_plugins"]["default"];
    const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

    {
      std::set<std::string> sp = cmpi.search_paths;
      EXPECT_EQ(sp.size(), 1);

      for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
      {
        EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
      }
    }

    {
      std::set<std::string> sl = cmpi.search_libraries;
      EXPECT_EQ(sl.size(), 2);

      for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
      {
        EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
      }
    }

    EXPECT_EQ(discrete_default_plugin.as<std::string>(), cmpi.discrete_plugin_infos.default_plugin);
    EXPECT_EQ(discrete_plugins.size(), cmpi.discrete_plugin_infos.plugins.size());

    EXPECT_EQ(continuous_default_plugin.as<std::string>(), cmpi.continuous_plugin_infos.default_plugin);
    EXPECT_EQ(continuous_plugins.size(), cmpi.continuous_plugin_infos.plugins.size());
  }

  {  // search_paths failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     failure: issue
                                   search_libraries:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   discrete_plugins:
                                     default: BulletDiscreteBVHManager
                                     plugins:
                                       BulletDiscreteBVHManager:
                                         class: BulletDiscreteBVHManagerFactory
                                       BulletDiscreteSimpleManager:
                                         class: BulletDiscreteSimpleManagerFactory
                                       FCLDiscreteBVHManager:
                                         class: FCLDiscreteBVHManagerFactory
                                   continuous_plugins:
                                     default: BulletCastBVHManager
                                     plugins:
                                       BulletCastBVHManager:
                                         class: BulletCastBVHManagerFactory
                                       BulletCastSimpleManager:
                                         class: BulletCastSimpleManagerFactory)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }

  {  // search_libraries failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     failure: issue
                                   discrete_plugins:
                                     default: BulletDiscreteBVHManager
                                     plugins:
                                       BulletDiscreteBVHManager:
                                         class: BulletDiscreteBVHManagerFactory
                                       BulletDiscreteSimpleManager:
                                         class: BulletDiscreteSimpleManagerFactory
                                       FCLDiscreteBVHManager:
                                         class: FCLDiscreteBVHManagerFactory
                                   continuous_plugins:
                                     default: BulletCastBVHManager
                                     plugins:
                                       BulletCastBVHManager:
                                         class: BulletCastBVHManagerFactory
                                       BulletCastSimpleManager:
                                         class: BulletCastSimpleManagerFactory)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }

  {  // missing discrete plugins failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   discrete_plugins:
                                     default: BulletDiscreteBVHManager
                                   continuous_plugins:
                                     default: BulletCastBVHManager
                                     plugins:
                                       BulletCastBVHManager:
                                         class: BulletCastBVHManagerFactory
                                       BulletCastSimpleManager:
                                         class: BulletCastSimpleManagerFactory)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }

  {  // discrete plugins is not map failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   discrete_plugins:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   continuous_plugins:
                                     default: BulletCastBVHManager
                                     plugins:
                                       BulletCastBVHManager:
                                         class: BulletCastBVHManagerFactory
                                       BulletCastSimpleManager:
                                         class: BulletCastSimpleManagerFactory)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }

  {  // missing continuous plugins failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   discrete_plugins:
                                     default: BulletDiscreteBVHManager
                                     plugins:
                                       BulletDiscreteBVHManager:
                                         class: BulletDiscreteBVHManagerFactory
                                       BulletDiscreteSimpleManager:
                                         class: BulletDiscreteSimpleManagerFactory
                                       FCLDiscreteBVHManager:
                                         class: FCLDiscreteBVHManagerFactory
                                   continuous_plugins:
                                     default: BulletCastBVHManager)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }

  {  // continuous plugins is not map failure
    std::string yaml_string = R"(contact_manager_plugins:
                                   search_paths:
                                     - /usr/local/lib
                                   search_libraries:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories
                                   discrete_plugins:
                                     default: BulletDiscreteBVHManager
                                     plugins:
                                       BulletDiscreteBVHManager:
                                         class: BulletDiscreteBVHManagerFactory
                                       BulletDiscreteSimpleManager:
                                         class: BulletDiscreteSimpleManagerFactory
                                       FCLDiscreteBVHManager:
                                         class: FCLDiscreteBVHManagerFactory
                                   continuous_plugins:
                                     - tesseract_collision_bullet_factories
                                     - tesseract_collision_fcl_factories)";
    YAML::Node plugin_config = YAML::Load(yaml_string);
    YAML::Node config = plugin_config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
    EXPECT_ANY_THROW(config.as<tesseract_common::ContactManagersPluginInfo>());  // NOLINT
  }
}

TEST(TesseractCommonUnit, TransformMapYamlUnit)  // NOLINT
{
  std::string yaml_string =
      R"(joints:
           joint_1:
             position:
               x: 1
               y: 2
               z: 3
             orientation:
               x: 0
               y: 0
               z: 0
               w: 1
           joint_2:
             position:
               x: 4
               y: 5
               z: 6
             orientation:
               x: 0
               y: 0
               z: 0
               w: 1)";

  {  // valid string
    YAML::Node node = YAML::Load(yaml_string);
    auto trans_map = node["joints"].as<tesseract_common::TransformMap>();
    EXPECT_EQ(trans_map.size(), 2);
    EXPECT_FALSE(trans_map.empty());
    EXPECT_TRUE(trans_map.find("joint_1") != trans_map.end());
    EXPECT_TRUE(trans_map.find("joint_2") != trans_map.end());
  }

  std::string bad_yaml_string =
      R"(joints:
           - joint_1:
               position:
                 x: 1
                 y: 2
                 z: 3
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1
           - joint_2:
               position:
                 x: 4
                 y: 5
                 z: 6
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1)";
  {  // invalid string
    YAML::Node node = YAML::Load(bad_yaml_string);
    EXPECT_ANY_THROW(node["joints"].as<tesseract_common::TransformMap>());  // NOLINT
  }
}

TEST(TesseractCommonUnit, CalibrationInfoYamlUnit)  // NOLINT
{
  std::string yaml_string =
      R"(calibration:
           joints:
             joint_1:
               position:
                 x: 1
                 y: 2
                 z: 3
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1
             joint_2:
               position:
                 x: 4
                 y: 5
                 z: 6
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1)";

  YAML::Node node = YAML::Load(yaml_string);
  auto cal_info = node[tesseract_common::CalibrationInfo::CONFIG_KEY].as<tesseract_common::CalibrationInfo>();
  EXPECT_FALSE(cal_info.empty());
  EXPECT_TRUE(cal_info.joints.find("joint_1") != cal_info.joints.end());
  EXPECT_TRUE(cal_info.joints.find("joint_2") != cal_info.joints.end());

  tesseract_common::CalibrationInfo cal_insert;
  EXPECT_TRUE(cal_insert.empty());
  cal_insert.insert(cal_info);
  EXPECT_FALSE(cal_insert.empty());
  EXPECT_TRUE(cal_insert.joints.find("joint_1") != cal_insert.joints.end());
  EXPECT_TRUE(cal_insert.joints.find("joint_2") != cal_insert.joints.end());

  cal_info.clear();
  EXPECT_TRUE(cal_info.empty());
}

TEST(TesseractCommonUnit, linkNamesPairUnit)  // NOLINT
{
  tesseract_common::LinkNamesPair p1 = tesseract_common::makeOrderedLinkPair("link_1", "link_2");
  tesseract_common::LinkNamesPair p2 = tesseract_common::makeOrderedLinkPair("link_2", "link_1");

  tesseract_common::PairHash hash;
  EXPECT_EQ(hash(p1), hash(p2));
}

/** @brief Tests calcRotationalError which return angle between [-PI, PI]*/
TEST(TesseractCommonUnit, calcRotationalError)  // NOLINT
{
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(M_PI - 0.0001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d rot_err = tesseract_common::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI - 0.0001, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6));

  pi_rot = identity * Eigen::AngleAxisd(-M_PI + 0.0001, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI - 0.0001, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));

  // Test greater than PI
  pi_rot = identity * Eigen::AngleAxisd(3 * M_PI_2, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));

  // Test less than than -PI
  pi_rot = identity * Eigen::AngleAxisd(-3 * M_PI_2, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6));

  // Test for angle between [0, PI]
  Eigen::Isometry3d pi_rot_plus = identity * Eigen::AngleAxisd(M_PI_2 + 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d pi_rot_minus = identity * Eigen::AngleAxisd(M_PI_2 - 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d pi_rot_delta = tesseract_common::calcRotationalError(pi_rot_plus.rotation()) -
                                 tesseract_common::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle between [-PI, 0]
  pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI_2 + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI_2 - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 0
  pi_rot_plus = identity * Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at PI
  pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI

  // Test for angle at -PI
  pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI

  // Test random axis
  for (int i = 0; i < 100; i++)
  {
    Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();

    // Avoid M_PI angle because this breaks down
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(100, -M_PI + 0.005, M_PI - 0.005);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // Avoid M_PI angle because this breaks down
    angles = Eigen::VectorXd::LinSpaced(100, M_PI + 0.005, 2 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // Avoid M_PI angle because this breaks down
    angles = Eigen::VectorXd::LinSpaced(100, -M_PI - 0.005, -2 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // These should fail
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI
    }
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI
    }
  }
}

/** @brief Tests calcRotationalError2 which return angle between [0, 2 * PI]*/
TEST(TesseractCommonUnit, calcRotationalError2)  // NOLINT
{
  auto check_axis = [](const Eigen::Vector3d& axis) {
    return (axis.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6) ||
            axis.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));
  };
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(3 * M_PI_2, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d rot_err = tesseract_common::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  pi_rot = identity * Eigen::AngleAxisd(0.0001, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), 0.0001, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test greater than 2 * PI
  pi_rot = identity * Eigen::AngleAxisd(3 * M_PI, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test lessthan than 0
  pi_rot = identity * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ());
  rot_err = tesseract_common::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test for angle between [0, 2 * PI]
  Eigen::Isometry3d pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d pi_rot_delta = tesseract_common::calcRotationalError2(pi_rot_plus.rotation()) -
                                 tesseract_common::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 0
  pi_rot_plus = identity * Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError2(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 2 * PI
  pi_rot_plus = identity * Eigen::AngleAxisd((2 * M_PI) + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd((2 * M_PI) - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta = tesseract_common::calcRotationalError2(pi_rot_plus.rotation()) -
                 tesseract_common::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test random axis
  for (int i = 0; i < 100; i++)
  {
    Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();

    // Avoid M_PI angle because this breaks down
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(1000, -5 * M_PI, 5 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = tesseract_common::calcRotationalError2(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = tesseract_common::calcRotationalError2(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < 0));
      EXPECT_FALSE((e1.norm() > 2 * M_PI));
      EXPECT_FALSE((e2.norm() < 0));
      EXPECT_FALSE((e2.norm() > 2 * M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }
  }
}

/** @brief Tests calcTransformError */
TEST(TesseractCommonUnit, calcTransformError)  // NOLINT
{
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  {  // X-Axis
    Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    Eigen::VectorXd err = tesseract_common::calcTransformError(identity, pi_rot);
    EXPECT_TRUE(err.head(3).isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(err.tail(3).isApprox(Eigen::Vector3d(M_PI_2, 0, 0)));
  }

  {  // Y-Axis
    Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    Eigen::VectorXd err = tesseract_common::calcTransformError(identity, pi_rot);
    EXPECT_TRUE(err.head(3).isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(err.tail(3).isApprox(Eigen::Vector3d(0, M_PI_2, 0)));
  }

  {  // Z-Axis
    Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    Eigen::VectorXd err = tesseract_common::calcTransformError(identity, pi_rot);
    EXPECT_TRUE(err.head(3).isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(err.tail(3).isApprox(Eigen::Vector3d(0, 0, M_PI_2)));
  }

  {  // Translation
    Eigen::Isometry3d pi_rot = identity * Eigen::Translation3d(1, 2, 3);
    Eigen::VectorXd err = tesseract_common::calcTransformError(identity, pi_rot);
    EXPECT_TRUE(err.head(3).isApprox(Eigen::Vector3d(1, 2, 3)));
    EXPECT_TRUE(err.tail(3).isApprox(Eigen::Vector3d::Zero()));
  }
}

/** @brief Tests calcTransformError */
TEST(TesseractCommonUnit, computeRandomColor)  // NOLINT
{
  Eigen::Vector4d color = tesseract_common::computeRandomColor();
  EXPECT_FALSE(color(0) < 0);
  EXPECT_FALSE(color(1) < 0);
  EXPECT_FALSE(color(2) < 0);
  EXPECT_FALSE(color(3) < 0);
  EXPECT_FALSE(color(0) > 1);
  EXPECT_FALSE(color(1) > 1);
  EXPECT_FALSE(color(2) > 1);
  EXPECT_FALSE(color(3) > 1);
}

/** @brief Tests calcTransformError */
TEST(TesseractCommonUnit, concat)  // NOLINT
{
  Eigen::Vector3d a(1, 2, 3);
  Eigen::Vector3d b(4, 5, 6);

  Eigen::VectorXd c = tesseract_common::concat(a, b);
  EXPECT_EQ(c.rows(), a.rows() + b.rows());
  EXPECT_TRUE(c.head(3).isApprox(a));
  EXPECT_TRUE(c.tail(3).isApprox(b));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
