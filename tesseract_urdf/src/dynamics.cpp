/**
 * @file dynamics.cpp
 * @brief Parse dynamics from xml string
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
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/dynamics.h>
#include <tesseract_scene_graph/joint.h>

tesseract_scene_graph::JointDynamics::Ptr tesseract_urdf::parseDynamics(const tinyxml2::XMLElement* xml_element,
                                                                        int /*version*/)
{
  if (xml_element->Attribute("damping") == nullptr && xml_element->Attribute("friction") == nullptr)
    std::throw_with_nested(std::runtime_error("Dynamics: Missing both attributes 'damping' and 'friction', remove tag "
                                              "or add attributes and values!"));

  auto dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();

  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute("damping", &(dynamics->damping));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Dynamics: Error parsing attribute 'damping'!"));

  if (status == tinyxml2::XML_NO_ATTRIBUTE)
    CONSOLE_BRIDGE_logDebug("Dynamics: Missing attribute 'damping', using default value 0!");

  status = xml_element->QueryDoubleAttribute("friction", &(dynamics->friction));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Dynamics: Error parsing attribute 'friction'!"));

  if (status == tinyxml2::XML_NO_ATTRIBUTE)
    CONSOLE_BRIDGE_logDebug("Dynamics: Missing attribute 'friction', using default value 0!");

  return dynamics;
}

tinyxml2::XMLElement*
tesseract_urdf::writeDynamics(const std::shared_ptr<const tesseract_scene_graph::JointDynamics>& dynamics,
                              tinyxml2::XMLDocument& doc)
{
  if (dynamics == nullptr)
    std::throw_with_nested(std::runtime_error("Dynamics is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("dynamics");

  xml_element->SetAttribute("damping", dynamics->damping);
  xml_element->SetAttribute("friction", dynamics->damping);

  return xml_element;
}
