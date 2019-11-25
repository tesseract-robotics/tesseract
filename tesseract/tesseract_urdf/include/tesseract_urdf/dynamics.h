/**
 * @file dynamics.h
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
#ifndef TESSERACT_URDF_DYNAMICS_H
#define TESSERACT_URDF_DYNAMICS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>

namespace tesseract_urdf
{
class DynamicsStatusCategory : public tesseract_common::StatusCategory
{
public:
  DynamicsStatusCategory() : name_("DynamicsStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case MissingAttributeDamping:
        return "Missing dynamics attribute 'damping', using default value 0!";
      case MissingAttributeFriction:
        return "Missing dynamics attribute 'friction', using default value 0!";
      case ErrorMissingAttributeDampingAndFriction:
        return "Missing both dynamics attribute 'damping' and 'friction', remove tag or add attributes and values";
      case ErrorParsingAttributeDamping:
        return "Error parsing dynamics attribute 'damping'!";
      case ErrorParsingAttributeFriction:
        return "Error parsing dynamics attribute 'friction'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MissingAttributeDamping = 2,
    MissingAttributeFriction = 1,
    Success = 0,
    ErrorMissingAttributeDampingAndFriction = -1,
    ErrorParsingAttributeDamping = -2,
    ErrorParsingAttributeFriction = -3
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::JointDynamics::Ptr& dynamics,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  dynamics = nullptr;
  auto status_cat = std::make_shared<DynamicsStatusCategory>();
  auto status_code = std::make_shared<tesseract_common::StatusCode>(DynamicsStatusCategory::Success, status_cat);

  if (xml_element->Attribute("damping") == nullptr && xml_element->Attribute("friction") == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(
        DynamicsStatusCategory::ErrorMissingAttributeDampingAndFriction, status_cat);

  dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();

  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute("damping", &(dynamics->damping));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(DynamicsStatusCategory::ErrorParsingAttributeDamping,
                                                          status_cat);

  if (status == tinyxml2::XML_NO_ATTRIBUTE)
    status_code =
        std::make_shared<tesseract_common::StatusCode>(DynamicsStatusCategory::MissingAttributeDamping, status_cat);

  status = xml_element->QueryDoubleAttribute("friction", &(dynamics->friction));
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(DynamicsStatusCategory::ErrorParsingAttributeFriction,
                                                          status_cat);

  if (status == tinyxml2::XML_NO_ATTRIBUTE)
    status_code =
        std::make_shared<tesseract_common::StatusCode>(DynamicsStatusCategory::MissingAttributeFriction, status_cat);

  return status_code;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_DYNAMICS_H
