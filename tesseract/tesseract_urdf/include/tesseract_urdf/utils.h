/**
 * @file utils.h
 * @brief Utils for parsing urdf
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_URDF_UTILS_H
#define TESSERACT_URDF_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_urdf
{
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
  boost::trim(value);
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
  boost::trim(value);
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

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_UTILS_H
