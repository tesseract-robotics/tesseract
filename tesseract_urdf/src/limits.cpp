/**
 * @file limits.cpp
 * @brief Parse limits from xml string
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

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/limits.h>
#include <tesseract_urdf/utils.h>

tesseract_scene_graph::JointLimits::Ptr tesseract_urdf::parseLimits(const tinyxml2::XMLElement* xml_element,
                                                                    int /*version*/)
{
  auto limits = std::make_shared<tesseract_scene_graph::JointLimits>();

  int status = xml_element->QueryDoubleAttribute("lower", &(limits->lower));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Limits: Missing or failed to parse attribute 'lower'!"));

  status = xml_element->QueryDoubleAttribute("upper", &(limits->upper));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Limits: Missing or failed to parse attribute 'upper'!"));

  if (xml_element->QueryDoubleAttribute("effort", &(limits->effort)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Limits: Missing or failed to parse attribute 'effort'!"));

  if (xml_element->QueryDoubleAttribute("velocity", &(limits->velocity)) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Limits: Missing or failed to parse attribute 'velocity'!"));

  status = xml_element->QueryDoubleAttribute("acceleration", &(limits->acceleration));
  if (status == tinyxml2::XML_NO_ATTRIBUTE)
    limits->acceleration = 0.5 * limits->velocity;
  else if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Limits: Failed to parse attribute 'acceleration'!"));

  return limits;
}

tinyxml2::XMLElement*
tesseract_urdf::writeLimits(const std::shared_ptr<const tesseract_scene_graph::JointLimits>& limits,
                            tinyxml2::XMLDocument& doc)
{
  if (limits == nullptr)
    std::throw_with_nested(std::runtime_error("Limits are nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("limit");

  // if upper and lower are both zero, don't write it.  This should only happen for continuous joints.
  if (!tesseract_common::almostEqualRelativeAndAbs(limits->lower, 0.0) ||
      !tesseract_common::almostEqualRelativeAndAbs(limits->upper, 0.0))
  {
    xml_element->SetAttribute("lower", toString(limits->lower).c_str());
    xml_element->SetAttribute("upper", toString(limits->upper).c_str());
  }

  // Always write effort & velocity
  xml_element->SetAttribute("effort", toString(limits->effort).c_str());
  xml_element->SetAttribute("velocity", toString(limits->velocity).c_str());

  // Write out nonzero acceleration (Tesseract-exclusive)
  if (!tesseract_common::almostEqualRelativeAndAbs(limits->acceleration, 0.0) &&
      !tesseract_common::almostEqualRelativeAndAbs(limits->acceleration, limits->velocity * 0.5))
    xml_element->SetAttribute("acceleration", toString(limits->acceleration).c_str());

  return xml_element;
}
