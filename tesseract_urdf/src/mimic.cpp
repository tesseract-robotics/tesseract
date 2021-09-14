/**
 * @file mimic.cpp
 * @brief Parse mimic from xml string
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <stdexcept>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/mimic.h>
#include <tesseract_scene_graph/joint.h>

tesseract_scene_graph::JointMimic::Ptr tesseract_urdf::parseMimic(const tinyxml2::XMLElement* xml_element,
                                                                  int /*version*/)
{
  auto m = std::make_shared<tesseract_scene_graph::JointMimic>();
  if (tesseract_common::QueryStringAttribute(xml_element, "joint", m->joint_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Mimic: Missing or failed to parse mimic attribute 'joint'!"));

  if (xml_element->Attribute("offset") == nullptr && xml_element->Attribute("multiplier") == nullptr)
    CONSOLE_BRIDGE_logDebug("Mimic: Missing attribute 'offset' and 'multiplier', using default value 0 and 1!");
  else if (xml_element->Attribute("offset") != nullptr && xml_element->Attribute("multiplier") == nullptr)
    CONSOLE_BRIDGE_logDebug("Mimic: Missing attribute 'multiplier', using default value 1!");
  else if (xml_element->Attribute("offset") == nullptr && xml_element->Attribute("multiplier") != nullptr)
    CONSOLE_BRIDGE_logDebug("Mimic: Missing attribute 'offset', using default value 1!");

  tinyxml2::XMLError s = xml_element->QueryDoubleAttribute("offset", &(m->offset));
  if (s != tinyxml2::XML_NO_ATTRIBUTE && s != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Mimic: Error parsing attribute 'offset'!"));

  s = xml_element->QueryDoubleAttribute("multiplier", &(m->multiplier));
  if (s != tinyxml2::XML_NO_ATTRIBUTE && s != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Mimic: Error parsing attribute 'multiplier'!"));

  return m;
}

tinyxml2::XMLElement* tesseract_urdf::writeMimic(const std::shared_ptr<const tesseract_scene_graph::JointMimic>& mimic,
                                                 tinyxml2::XMLDocument& doc)
{
  if (mimic == nullptr)
    std::throw_with_nested(std::runtime_error("Mimic Joint is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("mimic");

  xml_element->SetAttribute("joint", mimic->joint_name.c_str());
  xml_element->SetAttribute("offset", mimic->offset);
  xml_element->SetAttribute("multiplier", mimic->multiplier);

  return xml_element;
}
