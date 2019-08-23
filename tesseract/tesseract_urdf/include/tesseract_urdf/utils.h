/**
 * @file utils.h
 * @brief Utils for parsing urdf
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

inline tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value)
{
  if (xml_element->Value() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  value = std::string(xml_element->Value());
  boost::trim(value);
  return tinyxml2::XML_SUCCESS;
}

inline tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value)
{
  if (xml_attribute->Value() == nullptr)
    return tinyxml2::XML_WRONG_ATTRIBUTE_TYPE;

  value = std::string(xml_attribute->Value());
  boost::trim(value);
  return tinyxml2::XML_SUCCESS;
}

inline tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string& value)
{
    const tinyxml2::XMLAttribute* attribute = xml_element->FindAttribute(name);
    if (attribute == nullptr)
      return tinyxml2::XML_NO_ATTRIBUTE;

    return QueryStringValue(attribute, value);
}

inline std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value)
{
  std::string str = default_value;
  QueryStringAttribute(xml_element, name, str);
  return str;
}

}

#endif // TESSERACT_URDF_UTILS_H
