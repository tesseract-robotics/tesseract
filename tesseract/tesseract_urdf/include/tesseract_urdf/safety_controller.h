/**
 * @file safety_controller.h
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
#ifndef TESSERACT_URDF_SAFETY_CONTROLLER_H
#define TESSERACT_URDF_SAFETY_CONTROLLER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>

namespace tesseract_urdf
{
class SafetyStatusCategory : public tesseract_common::StatusCategory
{
public:
  SafetyStatusCategory() : name_("SafetyStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case MissingAttributeAllOptional:
        return "Missing safety attribute 'soft_upper_limit', 'soft_lower_limit', and 'k_position', using default value "
               "0, 0, and 1!";
      case MissingAttributeSoftUpperLimit:
        return "Missing safety attribute 'soft_upper_limit', using default value 0!";
      case MissingAttributeSoftLowerLimit:
        return "Missing safety attribute 'soft_lower_limit', using default value 0!";
      case MissingAttributeKPosition:
        return "Missing safety attribute 'k_position', using default value 0!";
      case ErrorAttributeKVelocity:
        return "Missing or failed to parse safety attribute 'k_velocity'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MissingAttributeAllOptional = 4,
    MissingAttributeSoftUpperLimit = 3,
    MissingAttributeSoftLowerLimit = 2,
    MissingAttributeKPosition = 1,
    Success = 0,
    ErrorAttributeKVelocity = -1
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::JointSafety::Ptr& safety,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  safety = nullptr;

  auto status_cat = std::make_shared<SafetyStatusCategory>();
  auto s = std::make_shared<tesseract_scene_graph::JointSafety>();
  if (xml_element->QueryDoubleAttribute("k_velocity", &(s->k_velocity)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(SafetyStatusCategory::ErrorAttributeKVelocity, status_cat);

  tesseract_common::StatusCode::Ptr status = nullptr;
  if (xml_element->Attribute("soft_upper_limit") == nullptr && xml_element->Attribute("soft_lower_limit") == nullptr &&
      xml_element->Attribute("k_position") == nullptr)
  {
    status = std::make_shared<tesseract_common::StatusCode>(
        SafetyStatusCategory::MissingAttributeAllOptional, status_cat, status);
  }
  else if (xml_element->Attribute("soft_upper_limit") == nullptr ||
           xml_element->Attribute("soft_lower_limit") == nullptr || xml_element->Attribute("k_position") == nullptr)
  {
    if (xml_element->Attribute("soft_upper_limit") == nullptr)
      status = std::make_shared<tesseract_common::StatusCode>(
          SafetyStatusCategory::MissingAttributeSoftUpperLimit, status_cat, status);

    if (xml_element->Attribute("soft_lower_limit") == nullptr)
      status = std::make_shared<tesseract_common::StatusCode>(
          SafetyStatusCategory::MissingAttributeSoftLowerLimit, status_cat, status);

    if (xml_element->Attribute("k_position") == nullptr)
      status = std::make_shared<tesseract_common::StatusCode>(
          SafetyStatusCategory::MissingAttributeKPosition, status_cat, status);
  }

  s->soft_upper_limit = 0;
  s->soft_lower_limit = 0;
  s->k_position = 0;
  xml_element->QueryDoubleAttribute("soft_upper_limit", &s->soft_upper_limit);
  xml_element->QueryDoubleAttribute("soft_lower_limit", &s->soft_lower_limit);
  xml_element->QueryDoubleAttribute("k_position", &s->k_position);

  safety = std::move(s);
  return std::make_shared<tesseract_common::StatusCode>(SafetyStatusCategory::Success, status_cat, status);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_SAFETY_CONTROLLER_H
