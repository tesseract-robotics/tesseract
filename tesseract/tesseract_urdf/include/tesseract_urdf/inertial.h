/**
 * @file inertial.h
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
#ifndef TESSERACT_URDF_INERTIAL_H
#define TESSERACT_URDF_INERTIAL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_urdf/origin.h>

namespace tesseract_urdf
{
class InertialStatusCategory : public tesseract_common::StatusCategory
{
public:
  InertialStatusCategory() : name_("InertialStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parssed inertial!";
      case MissingOriginElement:
        return "Missing optional inertial element 'origin'!";
      case ErrorParsingOrigin:
        return "Failed parsing inertial element 'origin'!";
      case ErrorMissingMassElement:
        return "Missing inertial element 'mass'!";
      case ErrorMassAttributeValue:
        return "Missing or failed parsing 'mass' attribute 'value'!";
      case ErrorMissingInertiaElement:
        return "Missing inertial element 'inertia'!";
      case ErrorInertiaAttributeIxx:
        return "Missing or failed parsing 'inertia' attribute 'ixx'!";
      case ErrorInertiaAttributeIxy:
        return "Missing or failed parsing 'inertia' attribute 'ixy'!";
      case ErrorInertiaAttributeIxz:
        return "Missing or failed parsing 'inertia' attribute 'ixz'!";
      case ErrorInertiaAttributeIyy:
        return "Missing or failed parsing 'inertia' attribute 'iyy'!";
      case ErrorInertiaAttributeIyz:
        return "Missing or failed parsing 'inertia' attribute 'iyz'!";
      case ErrorInertiaAttributeIzz:
        return "Missing or failed parsing 'inertia' attribute 'izz'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MissingOriginElement = 1,
    Success = 0,
    ErrorParsingOrigin = -1,
    ErrorMissingMassElement = -2,
    ErrorMassAttributeValue = -3,
    ErrorMissingInertiaElement = -4,
    ErrorInertiaAttributeIxx = -5,
    ErrorInertiaAttributeIxy = -6,
    ErrorInertiaAttributeIxz = -7,
    ErrorInertiaAttributeIyy = -8,
    ErrorInertiaAttributeIyz = -9,
    ErrorInertiaAttributeIzz = -10,
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::Inertial::Ptr& inertial,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int version)
{
  inertial = nullptr;
  auto status_cat = std::make_shared<InertialStatusCategory>();

  auto i = std::make_shared<tesseract_scene_graph::Inertial>();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    auto status = parse(i->origin, origin, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          InertialStatusCategory::ErrorParsingOrigin, status_cat, status);
  }

  const tinyxml2::XMLElement* mass = xml_element->FirstChildElement("mass");
  if (mass == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorMissingMassElement, status_cat);

  if (mass->QueryDoubleAttribute("value", &(i->mass)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorMassAttributeValue, status_cat);

  const tinyxml2::XMLElement* inertia = xml_element->FirstChildElement("inertia");
  if (inertia == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorMissingInertiaElement,
                                                          status_cat);

  if (inertia->QueryDoubleAttribute("ixx", &(i->ixx)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIxx, status_cat);

  if (inertia->QueryDoubleAttribute("ixy", &(i->ixy)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIxy, status_cat);

  if (inertia->QueryDoubleAttribute("ixz", &(i->ixz)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIxz, status_cat);

  if (inertia->QueryDoubleAttribute("iyy", &(i->iyy)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIyy, status_cat);

  if (inertia->QueryDoubleAttribute("iyz", &(i->iyz)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIyz, status_cat);

  if (inertia->QueryDoubleAttribute("izz", &(i->izz)) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::ErrorInertiaAttributeIzz, status_cat);

  inertial = std::move(i);
  return std::make_shared<tesseract_common::StatusCode>(InertialStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_INERTIAL_H
