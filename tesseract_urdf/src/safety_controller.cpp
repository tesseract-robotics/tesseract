/**
 * @file safety_controller.cpp
 * @brief Parse safety_controller from xml string
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

#include <tesseract_urdf/safety_controller.h>
#include <tesseract_scene_graph/joint.h>

tesseract_scene_graph::JointSafety::Ptr tesseract_urdf::parseSafetyController(const tinyxml2::XMLElement* xml_element,
                                                                              int /*version*/)
{
  auto s = std::make_shared<tesseract_scene_graph::JointSafety>();
  if (xml_element->QueryDoubleAttribute("k_velocity", &(s->k_velocity)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SafetyController: Missing or failed to parse attribute 'k_velocity'!"));

  if (xml_element->Attribute("soft_upper_limit") == nullptr && xml_element->Attribute("soft_lower_limit") == nullptr &&
      xml_element->Attribute("k_position") == nullptr)
  {
    CONSOLE_BRIDGE_logDebug("SafetyController: Missing attributes 'soft_upper_limit', 'soft_lower_limit', and "
                            "'k_position', using default value 0, 0, and 0!");
  }
  else if (xml_element->Attribute("soft_upper_limit") == nullptr ||
           xml_element->Attribute("soft_lower_limit") == nullptr || xml_element->Attribute("k_position") == nullptr)
  {
    if (xml_element->Attribute("soft_upper_limit") == nullptr)
      CONSOLE_BRIDGE_logDebug("SafetyController: Missing attribute 'soft_upper_limit', using default value 0!");

    if (xml_element->Attribute("soft_lower_limit") == nullptr)
      CONSOLE_BRIDGE_logDebug("SafetyController: Missing attribute 'soft_lower_limit', using default value 0!");

    if (xml_element->Attribute("k_position") == nullptr)
      CONSOLE_BRIDGE_logDebug("SafetyController: Missing attribute 'k_position', using default value 0!");
  }

  s->soft_upper_limit = 0;
  s->soft_lower_limit = 0;
  s->k_position = 0;
  xml_element->QueryDoubleAttribute("soft_upper_limit", &s->soft_upper_limit);
  xml_element->QueryDoubleAttribute("soft_lower_limit", &s->soft_lower_limit);
  xml_element->QueryDoubleAttribute("k_position", &s->k_position);

  return s;
}
