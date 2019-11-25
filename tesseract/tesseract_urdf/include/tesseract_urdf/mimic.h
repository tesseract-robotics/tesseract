/**
 * @file mimic.h
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
#ifndef TESSERACT_URDF_MIMIC_H
#define TESSERACT_URDF_MIMIC_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
{
class MimicStatusCategory : public tesseract_common::StatusCategory
{
public:
  MimicStatusCategory() : name_("MimicStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case MissingAttributeOffsetAndMultiplier:
        return "Missing mimic attribute 'offset' and 'multiplier', using default value 0 and 1!";
      case MissingAttributeOffset:
        return "Missing mimic attribute 'offset', using default value 0!";
      case MissingAttributeMultiplier:
        return "Missing mimic attribute 'multiplier', using default value 1!";
      case ErrorAttributeJoint:
        return "Missing or failed to parse mimic attribute 'joint'!";
      case ErrorParsingAttributeOffset:
        return "Error parsing mimic attribute 'offset'!";
      case ErrorParsingAttributeMultiplier:
        return "Error parsing mimic attribute 'multiplier'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MissingAttributeOffsetAndMultiplier = 3,
    MissingAttributeOffset = 2,
    MissingAttributeMultiplier = 1,
    Success = 0,
    ErrorAttributeJoint = -1,
    ErrorParsingAttributeOffset = -2,
    ErrorParsingAttributeMultiplier = -3
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::JointMimic::Ptr& mimic,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  mimic = nullptr;
  auto status_cat = std::make_shared<MimicStatusCategory>();

  auto m = std::make_shared<tesseract_scene_graph::JointMimic>();
  if (QueryStringAttribute(xml_element, "joint", m->joint_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::ErrorAttributeJoint, status_cat);

  auto status = std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::Success, status_cat);
  if (xml_element->Attribute("offset") == nullptr && xml_element->Attribute("multiplier") == nullptr)
    status = std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::MissingAttributeOffsetAndMultiplier,
                                                            status_cat);
  else if (xml_element->Attribute("offset") != nullptr && xml_element->Attribute("multiplier") == nullptr)
    status =
        std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::MissingAttributeMultiplier, status_cat);
  else if (xml_element->Attribute("offset") == nullptr && xml_element->Attribute("multiplier") != nullptr)
    status = std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::MissingAttributeOffset, status_cat);

  tinyxml2::XMLError s = xml_element->QueryDoubleAttribute("offset", &(m->offset));
  if (s != tinyxml2::XML_NO_ATTRIBUTE && s != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::ErrorParsingAttributeOffset, status_cat);

  s = xml_element->QueryDoubleAttribute("multiplier", &(m->multiplier));
  if (s != tinyxml2::XML_NO_ATTRIBUTE && s != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(MimicStatusCategory::ErrorParsingAttributeMultiplier,
                                                          status_cat);

  mimic = std::move(m);
  return status;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_MIMIC_H
