/**
 * @file utils.cpp
 * @brief Common Tesseract Utility Functions
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <string>
#include <type_traits>
#include <console_bridge/console.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

namespace tesseract_common
{
// explicit Instantiation
template bool toNumeric<double>(const std::string&, double&);
template bool toNumeric<float>(const std::string&, float&);
template bool toNumeric<int>(const std::string&, int&);
template bool toNumeric<long>(const std::string&, long&);
template bool isIdentical<std::string>(const std::vector<std::string>&,
                                       const std::vector<std::string>&,
                                       bool,
                                       const std::function<bool(const std::string&, const std::string&)>&,
                                       const std::function<bool(const std::string&, const std::string&)>&);
template bool isIdentical<Eigen::Index>(const std::vector<Eigen::Index>&,
                                        const std::vector<Eigen::Index>&,
                                        bool,
                                        const std::function<bool(const Eigen::Index&, const Eigen::Index&)>&,
                                        const std::function<bool(const Eigen::Index&, const Eigen::Index&)>&);

// Similar to rethrow_if_nested
// but does nothing instead of calling std::terminate
// when std::nested_exception is nullptr.
template <typename E>
std::enable_if_t<!std::is_polymorphic<E>::value> my_rethrow_if_nested(const E&)
{
}

template <typename E>
std::enable_if_t<std::is_polymorphic<E>::value> my_rethrow_if_nested(const E& e)
{
  const auto* p = dynamic_cast<const std::nested_exception*>(std::addressof(e));
  if (p && p->nested_ptr())
    p->rethrow_nested();
}

std::string fileToString(const tesseract_common::fs::path& filepath)
{
  std::ifstream ifs(filepath.c_str());
  std::string contents;

  ifs.seekg(0, std::ios::end);
  contents.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);

  contents.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
  ifs.close();
  return contents;
}

// LCOV_EXCL_START
// These are tested in both tesseract_kinematics and tesseract_state_solver
void twistChangeRefPoint(Eigen::Ref<Eigen::VectorXd> twist, const Eigen::Ref<const Eigen::Vector3d>& ref_point)
{
  twist(0) += twist(4) * ref_point(2) - twist(5) * ref_point(1);
  twist(1) += twist(5) * ref_point(0) - twist(3) * ref_point(2);
  twist(2) += twist(3) * ref_point(1) - twist(4) * ref_point(0);
}

void twistChangeBase(Eigen::Ref<Eigen::VectorXd> twist, const Eigen::Isometry3d& change_base)
{
  twist.head(3) = change_base.linear() * twist.head(3);
  twist.tail(3) = change_base.linear() * twist.tail(3);
}

void jacobianChangeBase(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Isometry3d& change_base)
{
  assert(jacobian.rows() == 6);
  for (int i = 0; i < jacobian.cols(); i++)
    twistChangeBase(jacobian.col(i), change_base);
}

void jacobianChangeRefPoint(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Ref<const Eigen::Vector3d>& ref_point)
{
  assert(jacobian.rows() == 6);
  for (int i = 0; i < jacobian.cols(); i++)
    twistChangeRefPoint(jacobian.col(i), ref_point);
}
// LCOV_EXCL_STOP

Eigen::VectorXd concat(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  Eigen::VectorXd out(a.size() + b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

Eigen::Vector3d calcRotationalError(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
  Eigen::Quaterniond q(R);
  Eigen::AngleAxisd r12(q);

  // Eigen angle axis flips the sign of axis so rotation is always positive which is
  // not ideal for numerical differentiation.
  int s = (q.vec().dot(r12.axis()) < 0) ? -1 : 1;

  // Make sure that the angle is on [-pi, pi]
  const static double two_pi = 2.0 * M_PI;
  double angle = s * r12.angle();
  Eigen::Vector3d axis = s * r12.axis();
  angle = copysign(fmod(fabs(angle), two_pi), angle);
  if (angle < -M_PI)
    angle += two_pi;
  else if (angle > M_PI)
    angle -= two_pi;

  assert(std::abs(angle) <= M_PI);

  return axis * angle;
}

Eigen::Vector3d calcRotationalError2(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
  Eigen::Quaterniond q(R);
  Eigen::AngleAxisd r12(q);

  // Eigen angle axis flips the sign of axis so rotation is always positive which is
  // not ideal for numerical differentiation.
  int s = (q.vec().dot(r12.axis()) < 0) ? -1 : 1;

  // Make sure that the angle is on [0, 2 * pi]
  const static double two_pi = 2.0 * M_PI;
  double angle = s * r12.angle();
  Eigen::Vector3d axis = s * r12.axis();
  angle = copysign(fmod(fabs(angle), two_pi), angle);
  if (angle < 0)
    angle += two_pi;
  else if (angle > two_pi)
    angle -= two_pi;

  assert(angle <= two_pi && angle >= 0);

  return axis * angle;
}

Eigen::VectorXd calcTransformError(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2)
{
  Eigen::Isometry3d pose_err = t1.inverse() * t2;
  return concat(pose_err.translation(), calcRotationalError(pose_err.rotation()));
}

Eigen::Vector4d computeRandomColor()
{
  Eigen::Vector4d c;
  c.setZero();
  c[3] = 1;
  while (almostEqualRelativeAndAbs(c[0], c[1]) || almostEqualRelativeAndAbs(c[2], c[1]) ||
         almostEqualRelativeAndAbs(c[2], c[0]))
  {
    c[0] = (rand() % 100) / 100.0;
    c[1] = (rand() % 100) / 100.0;
    c[2] = (rand() % 100) / 100.0;
  }
  return c;
}

void printNestedException(const std::exception& e, int level)  // NOLINT(misc-no-recursion)
{
  std::cerr << std::string(static_cast<unsigned>(2 * level), ' ') << "exception: " << e.what() << std::endl;
  try
  {
    my_rethrow_if_nested(e);
  }
  catch (const std::exception& e)
  {
    printNestedException(e, level + 1);
  }
  catch (...)
  {
  }
}

std::string getTempPath() { return fs::temp_directory_path().string() + std::string(1, fs::path::preferred_separator); }

bool isNumeric(const std::string& s)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  double out{ 0 };
  ss >> out;

  return !(ss.fail() || !ss.eof());
}

bool isNumeric(const std::vector<std::string>& sv)
{
  return std::all_of(sv.cbegin(), sv.cend(), [](const std::string& s) { return isNumeric(s); });
}

Eigen::VectorXd generateRandomNumber(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  Eigen::VectorXd joint_values;
  joint_values.resize(limits.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    std::uniform_real_distribution<double> sample(limits(i, 0), limits(i, 1));
    joint_values(i) = sample(mersenne);
  }
  return joint_values;
}

void ltrim(std::string& s) { s.erase(0, s.find_first_not_of(" \n\r\t\f\v")); }

void rtrim(std::string& s) { s.erase(s.find_last_not_of(" \n\r\t\f\v") + 1); }

void trim(std::string& s)
{
  ltrim(s);
  rtrim(s);
}

std::string getTimestampString()
{
  std::ostringstream oss;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  return oss.str();
}

void reorder(Eigen::Ref<Eigen::VectorXd> v, std::vector<Eigen::Index> order)
{
  assert(v.rows() == static_cast<Eigen::Index>(order.size()));
  // for all elements to put in place
  for (std::size_t i = 0; i < order.size() - 1; ++i)
  {
    if (order[i] == static_cast<Eigen::Index>(i))
      continue;

    size_t j{ 0 };
    for (j = i + 1; j < order.size(); ++j)
    {
      if (order[j] == static_cast<Eigen::Index>(i))
        break;
    }
    std::swap(v[static_cast<Eigen::Index>(i)], v[order[i]]);
    std::swap(order[i], order[j]);
  }
}

tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value)
{
  if (xml_element->Value() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  value = std::string(xml_element->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringText(const tinyxml2::XMLElement* xml_element, std::string& text)
{
  if (xml_element->GetText() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  text = std::string(xml_element->GetText());
  trim(text);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value)
{
  if (xml_attribute->Value() == nullptr)
    return tinyxml2::XML_WRONG_ATTRIBUTE_TYPE;

  value = std::string(xml_attribute->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string& value)
{
  const tinyxml2::XMLAttribute* attribute = xml_element->FindAttribute(name);
  if (attribute == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  return QueryStringValue(attribute, value);
}

std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value)
{
  std::string str = std::move(default_value);
  QueryStringAttribute(xml_element, name, str);
  return str;
}

tinyxml2::XMLError QueryStringAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                std::string& value)
{
  tinyxml2::XMLError status = QueryStringAttribute(xml_element, name, value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

tinyxml2::XMLError QueryDoubleAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                double& value)
{
  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute(name, &value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

tinyxml2::XMLError QueryIntAttributeRequired(const tinyxml2::XMLElement* xml_element, const char* name, int& value)
{
  tinyxml2::XMLError status = xml_element->QueryIntAttribute(name, &value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

bool almostEqualRelativeAndAbs(double a, double b, double max_diff, double max_rel_diff)
{
  double diff = std::fabs(a - b);
  if (diff <= max_diff)
    return true;

  a = std::fabs(a);
  b = std::fabs(b);
  double largest = (b > a) ? b : a;

  return (diff <= largest * max_rel_diff);
}

bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               double max_diff,
                               double max_rel_diff)
{
  const auto eigen_max_diff = Eigen::VectorXd::Constant(v1.size(), max_diff);
  const auto eigen_max_rel_diff = Eigen::VectorXd::Constant(v1.size(), max_rel_diff);
  // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.UndefReturn)
  return almostEqualRelativeAndAbs(v1, v2, eigen_max_diff, eigen_max_rel_diff);
}

bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               const Eigen::Ref<const Eigen::VectorXd>& max_diff,
                               const Eigen::Ref<const Eigen::VectorXd>& max_rel_diff)
{
  if (v1.size() == 0 && v2.size() == 0)
    return true;
  if (v1.size() != v2.size() || v1.size() != max_diff.size() || v1.size() != max_rel_diff.size())
    return false;

  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a1 = v1.array();
  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a2 = v2.array();
  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> md = max_diff.array();
  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> mrd = max_rel_diff.array();

  auto diff_abs = (a1 - a2).abs();
  double diff = diff_abs.maxCoeff();
  if ((diff <= md).all())
    return true;

  return (diff_abs <= (mrd * a1.abs().max(a2.abs()))).all();
}

std::vector<std::string> getAllowedCollisions(const std::vector<std::string>& link_names,
                                              const AllowedCollisionEntries& acm_entries,
                                              bool remove_duplicates)
{
  std::vector<std::string> results;
  results.reserve(acm_entries.size());

  for (const auto& entry : acm_entries)
  {
    const std::string link_1 = entry.first.first;
    const std::string link_2 = entry.first.second;

    // If the first entry is one of the links we were looking for
    if (std::find(link_names.begin(), link_names.end(), link_1) != link_names.end())
      // If it hasn't already been added or remove_duplicates is disabled
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_2) == results.end()))
        results.push_back(link_2);

    if (std::find(link_names.begin(), link_names.end(), link_2) != link_names.end())
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_1) == results.end()))
        results.push_back(link_1);
  }
  return results;
}

}  // namespace tesseract_common
