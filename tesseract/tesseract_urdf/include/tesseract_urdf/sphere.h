/**
 * @file sphere.h
 * @brief Parse sphere from xml string
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
#ifndef TESSERACT_URDF_SPHERE_H
#define TESSERACT_URDF_SPHERE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/sphere.h>

namespace tesseract_urdf
{
class SphereStatusCategory : public tesseract_common::StatusCategory
{
public:
  SphereStatusCategory() : name_("SphereStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case ErrorAttributeRadius:
        return "Missing or failed parsing sphere attribute radius!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeRadius = -1
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_geometry::Sphere::Ptr& sphere,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  sphere = nullptr;
  auto status_cat = std::make_shared<SphereStatusCategory>();

  double radius;
  if (xml_element->QueryDoubleAttribute("radius", &(radius)) != tinyxml2::XML_SUCCESS || !(radius > 0))
    return std::make_shared<tesseract_common::StatusCode>(SphereStatusCategory::ErrorAttributeRadius, status_cat);

  sphere = std::make_shared<tesseract_geometry::Sphere>(radius);
  return std::make_shared<tesseract_common::StatusCode>(SphereStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_SPHERE_H
