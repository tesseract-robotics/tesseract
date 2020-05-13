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
#include <ctime>
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{

/** @brief Random number generator */
inline std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };

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
inline Eigen::VectorXd generateRandomNumber(Eigen::MatrixX2d limits)
{
  Eigen::VectorXd joint_values;
  joint_values.resize(limits.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    std::uniform_real_distribution<double> sample{limits(i, 0), limits(i, 1)};
    joint_values(i) = sample(mersenne);
  }
  return joint_values;
}

/**
 * @brief Left trim string
 * @param s The string to left trim
 */
inline void ltrim(std::string& s)
{
  s.erase(0, s.find_first_not_of(" \n\r\t\f\v"));
}

/**
 * @brief Right trim string
 * @param s The string to right trim
 */
inline void rtrim(std::string& s)
{
  s.erase(s.find_last_not_of(" \n\r\t\f\v") + 1);
}


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

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
