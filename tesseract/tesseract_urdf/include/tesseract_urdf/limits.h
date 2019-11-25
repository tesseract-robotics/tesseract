/**
 * @file limits.h
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
#ifndef TESSERACT_URDF_LIMITS_H
#define TESSERACT_URDF_LIMITS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>

namespace tesseract_urdf
{
class LimitsStatusCategory : public tesseract_common::StatusCategory
{
public:
  LimitsStatusCategory() : name_("LimitsStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case ErrorAttributeLower:
        return "Missing or failed to parse limits attribute 'lower'!";
      case ErrorAttributeUpper:
        return "Missing or failed to parse limits attribute 'upper'!";
      case ErrorAttributeEffort:
        return "Missing or failed to parse limits attribute 'effort'!";
      case ErrorAttributeVelocity:
        return "Missing or failed to parse limits attribute 'velocity'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeLower = -1,
    ErrorAttributeUpper = -2,
    ErrorAttributeEffort = -3,
    ErrorAttributeVelocity = -4
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::JointLimits::Ptr& limits,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  limits = nullptr;

  tesseract_common::StatusCategory::Ptr status_cat = std::make_shared<LimitsStatusCategory>();
  auto l = std::make_shared<tesseract_scene_graph::JointLimits>();

  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute("lower", &(l->lower));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(LimitsStatusCategory::ErrorAttributeLower, status_cat);

  status = xml_element->QueryDoubleAttribute("upper", &(l->upper));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(LimitsStatusCategory::ErrorAttributeUpper, status_cat);

  if (xml_element->QueryDoubleAttribute("effort", &(l->effort)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(LimitsStatusCategory::ErrorAttributeEffort, status_cat);

  if (xml_element->QueryDoubleAttribute("velocity", &(l->velocity)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(LimitsStatusCategory::ErrorAttributeVelocity, status_cat);

  limits = l;
  return std::make_shared<tesseract_common::StatusCode>(LimitsStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_LIMITS_H
