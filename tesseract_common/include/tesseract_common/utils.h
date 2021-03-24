/**
 * @file utils.h
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
#ifndef TESSERACT_COMMON_UTILS_H
#define TESSERACT_COMMON_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <random>
#include <iomanip>
#include <ctime>
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>
#include <tinyxml2.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_common
{
#if __cplusplus < 201703L
/** @brief Random number generator */
static std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#else
/** @brief Random number generator */
inline std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#endif

/**
 * @brief Get the host temp directory path
 * @return The host temp directory path
 */
inline std::string getTempPath()
{
  return fs::temp_directory_path().string() + std::string(1, fs::path::preferred_separator);
}

/**
 * @brief Determine if a string is a number
 * @param s The string to evaluate
 * @return True if numeric, otherwise false.
 */
inline bool isNumeric(const std::string& s)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  double out;
  ss >> out;

  return !(ss.fail() || !ss.eof());
}

inline bool isNumeric(const std::vector<std::string>& sv)
{
  for (const auto& s : sv)
  {
    if (!isNumeric(s))
      return false;
  }

  return true;
}

/**
 * @brief Convert a string to a numeric value type
 * @param s The string to be converted
 * @param value The value to be loaded with coverted string
 * @return True if successful, otherwise false
 */
template <typename FloatType>
inline bool toNumeric(const std::string& s, FloatType& value)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  FloatType out;
  ss >> out;

  if (ss.fail() || !ss.eof())
    return false;

  value = out;
  return true;
}

/**
 * @brief Given a set of limits it will generate a vector of randon numbers between the limit.
 * @param limits The limits to generated numbers based on.
 * @return A vector of random numbers between the provided limits.
 */
inline Eigen::VectorXd generateRandomNumber(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
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

/**
 * @brief Left trim string
 * @param s The string to left trim
 */
inline void ltrim(std::string& s) { s.erase(0, s.find_first_not_of(" \n\r\t\f\v")); }

/**
 * @brief Right trim string
 * @param s The string to right trim
 */
inline void rtrim(std::string& s) { s.erase(s.find_last_not_of(" \n\r\t\f\v") + 1); }

/**
 * @brief Trim left and right of string
 * @param s The string to trim
 */
inline void trim(std::string& s)
{
  ltrim(s);
  rtrim(s);
}

/**
 * @brief Check if two vector of strings are identical
 * @param vec1 Vector strings
 * @param vec2 Vector strings
 * @param ordered If true order is relavent, othwise if false order is not relavent
 */
inline bool isIdentical(const std::vector<std::string>& vec1, const std::vector<std::string>& vec2, bool ordered = true)
{
  if (ordered)
    return std::equal(vec1.begin(), vec1.end(), vec2.begin());

  std::vector<std::string> v1 = vec1;
  std::vector<std::string> v2 = vec2;
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  return std::equal(v1.begin(), v1.end(), v2.begin());
}

/**
 * @brief Get Timestamp string
 * @return Timestamp string
 */
inline std::string getTimestampString()
{
  std::ostringstream oss;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  return oss.str();
}

/**
 * @brief Query a string value from xml element
 * @param xml_element The xml element to query string from
 * @param value The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
inline tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value)
{
  if (xml_element->Value() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  value = std::string(xml_element->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

/**
 * @brief Query a string Text from xml element
 * @param xml_element The xml element to query string from
 * @param test The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
inline tinyxml2::XMLError QueryStringText(const tinyxml2::XMLElement* xml_element, std::string& text)
{
  if (xml_element->GetText() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  text = std::string(xml_element->GetText());
  trim(text);
  return tinyxml2::XML_SUCCESS;
}

/**
 * @brief Query a string value from xml attribute
 * @param xml_attribute The xml attribute to query string from
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
inline tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value)
{
  if (xml_attribute->Value() == nullptr)
    return tinyxml2::XML_WRONG_ATTRIBUTE_TYPE;

  value = std::string(xml_attribute->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

/**
 * @brief Query a string attribute from an xml element
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
inline tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element,
                                               const char* name,
                                               std::string& value)
{
  const tinyxml2::XMLAttribute* attribute = xml_element->FindAttribute(name);
  if (attribute == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  return QueryStringValue(attribute, value);
}

/**
 * @brief Get string attribute if exist. If it does not exist it returns the default value.
 * @param xml_element The xml element to attempt to parse attribute
 * @param name The name of the attribute
 * @param default_value The default value to return if attribute does not exist
 * @return Parsed string attribute or default value if attribute does not exist.
 */
inline std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value)
{
  std::string str = std::move(default_value);
  QueryStringAttribute(xml_element, name, str);
  return str;
}

/**
 * @brief Query a string attribute from an xml element and print error log
 *
 * This is the same QueryStringAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
inline tinyxml2::XMLError QueryStringAttributeRequired(const tinyxml2::XMLElement* xml_element,
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

/**
 * @brief Query a double attribute from an xml element and print error log
 *
 * This is the same QueryDoubleAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
inline tinyxml2::XMLError QueryDoubleAttributeRequired(const tinyxml2::XMLElement* xml_element,
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

/**
 * @brief Query a int attribute from an xml element and print error log
 *
 * This is the same QueryIntAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
inline tinyxml2::XMLError QueryIntAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                    const char* name,
                                                    int& value)
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

/**
 * @brief Check if two double are relatively equal
 * @details The max_diff is for handling when comparing numbers near zero
 * @param a Double
 * @param b Double
 * @param max_diff The max diff when comparing std::abs(a - b) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff std::abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relativily equal, otherwise false.
 */
inline bool almostEqualRelativeAndAbs(double a,
                                      double b,
                                      double max_diff,
                                      double max_rel_diff = std::numeric_limits<double>::epsilon())
{
  double diff = std::fabs(a - b);
  if (diff <= max_diff)
    return true;

  a = std::fabs(a);
  b = std::fabs(b);
  double largest = (b > a) ? b : a;

  return (diff <= largest * max_rel_diff);
}

/**
 * @brief Check if two Eigen VectorXd are relatively and absolute equal
 * @details
 *    This is a vectorized implementation of the function above.
 *    The max_diff is for handling when comparing numbers near zero
 * @param a A vector of double
 * @param b A vector of double
 * @param max_diff The max diff when comparing max(abs(a - b)) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relativily equal, otherwise false.
 */
inline bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                                      const Eigen::Ref<const Eigen::VectorXd>& v2,
                                      double max_diff,
                                      double max_rel_diff = std::numeric_limits<double>::epsilon())
{
  if (v1.size() == 0 && v2.size() == 0)
    return true;
  if (v1.size() != v2.size())
    return false;

  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a1 = v1.array();
  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a2 = v2.array();

  auto diff_abs = (a1 - a2).abs();
  double diff = diff_abs.maxCoeff();
  if (diff <= max_diff)
    return true;

  return (diff_abs <= (max_rel_diff * a1.abs().max(a2.abs()))).all();
}

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
