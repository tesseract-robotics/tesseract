/**
 * @file cylinder.h
 * @brief Parse cylinder from xml string
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
#ifndef TESSERACT_URDF_CYLINDER_H
#define TESSERACT_URDF_CYLINDER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/cylinder.h>

namespace tesseract_urdf
{
class CylinderStatusCategory : public tesseract_common::StatusCategory
{
public:
  CylinderStatusCategory() : name_("CylinderStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case ErrorAttributeLength:
        return "Missing or failed parsing cylinder attribute length!";
      case ErrorAttributeRadius:
        return "Missing or failed parsing cylinder attribute radius!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeLength = -1,
    ErrorAttributeRadius = -2
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_geometry::Cylinder::Ptr& cylinder,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  cylinder = nullptr;
  auto status_cat = std::make_shared<CylinderStatusCategory>();

  double r, l;
  if (xml_element->QueryDoubleAttribute("length", &(l)) != tinyxml2::XML_SUCCESS || !(l > 0))
    return std::make_shared<tesseract_common::StatusCode>(CylinderStatusCategory::ErrorAttributeLength, status_cat);

  if (xml_element->QueryDoubleAttribute("radius", &(r)) != tinyxml2::XML_SUCCESS || !(r > 0))
    return std::make_shared<tesseract_common::StatusCode>(CylinderStatusCategory::ErrorAttributeRadius, status_cat);

  cylinder = std::make_shared<tesseract_geometry::Cylinder>(r, l);
  return std::make_shared<tesseract_common::StatusCode>(CylinderStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_CYLINDER_H
