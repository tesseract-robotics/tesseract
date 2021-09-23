/**
 * @file inertial.cpp
 * @brief Parse inertial from xml string
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
#include <stdexcept>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/inertial.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_scene_graph/link.h>

tesseract_scene_graph::Inertial::Ptr tesseract_urdf::parseInertial(const tinyxml2::XMLElement* xml_element, int version)
{
  auto inertial = std::make_shared<tesseract_scene_graph::Inertial>();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      inertial->origin = parseOrigin(origin, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Inertial: Failed parsing element 'origin'!"));
    }
  }

  const tinyxml2::XMLElement* mass = xml_element->FirstChildElement("mass");
  if (mass == nullptr)
    std::throw_with_nested(std::runtime_error("Inertial: Missing element 'mass'!"));

  if (mass->QueryDoubleAttribute("value", &(inertial->mass)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing 'mass' attribute 'value'!"));

  const tinyxml2::XMLElement* inertia = xml_element->FirstChildElement("inertia");
  if (inertia == nullptr)
    std::throw_with_nested(std::runtime_error("Inertial: Missing element 'inertia'!"));

  if (inertia->QueryDoubleAttribute("ixx", &(inertial->ixx)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'ixx'!"));

  if (inertia->QueryDoubleAttribute("ixy", &(inertial->ixy)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'ixy'!"));

  if (inertia->QueryDoubleAttribute("ixz", &(inertial->ixz)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'ixz'!"));

  if (inertia->QueryDoubleAttribute("iyy", &(inertial->iyy)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'iyy'!"));

  if (inertia->QueryDoubleAttribute("iyz", &(inertial->iyz)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'iyz'!"));

  if (inertia->QueryDoubleAttribute("izz", &(inertial->izz)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Inertial: Missing or failed parsing attribute 'izz'!"));

  return inertial;
}

tinyxml2::XMLElement*
tesseract_urdf::writeInertial(const std::shared_ptr<const tesseract_scene_graph::Inertial>& inertial,
                              tinyxml2::XMLDocument& doc)
{
  if (inertial == nullptr)
    std::throw_with_nested(std::runtime_error("Inertial is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("inertial");

  tinyxml2::XMLElement* xml_origin = writeOrigin(inertial->origin, doc);
  xml_element->InsertEndChild(xml_origin);

  tinyxml2::XMLElement* xml_mass = doc.NewElement("mass");
  xml_mass->SetAttribute("value", inertial->mass);

  tinyxml2::XMLElement* xml_inertia = doc.NewElement("inertia");
  xml_inertia->SetAttribute("ixx", inertial->ixx);
  xml_inertia->SetAttribute("ixy", inertial->ixy);
  xml_inertia->SetAttribute("ixz", inertial->ixz);
  xml_inertia->SetAttribute("iyy", inertial->iyy);
  xml_inertia->SetAttribute("iyz", inertial->iyz);
  xml_inertia->SetAttribute("izz", inertial->izz);

  xml_element->InsertEndChild(xml_mass);
  xml_element->InsertEndChild(xml_inertia);
  return xml_element;
}
