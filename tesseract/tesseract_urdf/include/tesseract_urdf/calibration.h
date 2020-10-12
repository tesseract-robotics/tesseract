/**
 * @file calibration.h
 * @brief Parse calibration from xml string
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
#ifndef TESSERACT_URDF_CALIBRATION_H
#define TESSERACT_URDF_CALIBRATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>

namespace tesseract_urdf
{
class CalibrationStatusCategory : public tesseract_common::StatusCategory
{
public:
  CalibrationStatusCategory() : name_("CalibrationStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case MissingAttributeRising:
        return "Missing calibration attribute 'rising', using default value 0!";
      case MissingAttributeFalling:
        return "Missing calibration attribute 'falling', using default value 0!";
      case ErrorMissingAttributeRisingAndFalling:
        return "Missing calibration both attribute 'rising' and 'falling', either remove tag add attributes and values";
      case ErrorParsingAttributeRising:
        return "Error parsing calibration attribute 'rising'!";
      case ErrorParsingAttributeFalling:
        return "Error parsing calibration attribute 'falling'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MissingAttributeRising = 2,
    MissingAttributeFalling = 1,
    Success = 0,
    ErrorMissingAttributeRisingAndFalling = -1,
    ErrorParsingAttributeRising = -2,
    ErrorParsingAttributeFalling = -3
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::JointCalibration::Ptr& calibration,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  calibration = nullptr;
  auto status_cat = std::make_shared<CalibrationStatusCategory>();

  if (xml_element->Attribute("rising") == nullptr && xml_element->Attribute("falling") == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(
        CalibrationStatusCategory::ErrorMissingAttributeRisingAndFalling, status_cat);

  calibration = std::make_shared<tesseract_scene_graph::JointCalibration>();
  auto status_code = std::make_shared<tesseract_common::StatusCode>(CalibrationStatusCategory::Success, status_cat);
  if (xml_element->Attribute("rising") == nullptr && xml_element->Attribute("falling") != nullptr)
    status_code =
        std::make_shared<tesseract_common::StatusCode>(CalibrationStatusCategory::MissingAttributeRising, status_cat);

  if (xml_element->Attribute("rising") != nullptr && xml_element->Attribute("falling") == nullptr)
    status_code =
        std::make_shared<tesseract_common::StatusCode>(CalibrationStatusCategory::MissingAttributeFalling, status_cat);

  auto xml_status = xml_element->QueryDoubleAttribute("rising", &(calibration->rising));
  if (xml_status != tinyxml2::XML_NO_ATTRIBUTE && xml_status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(CalibrationStatusCategory::ErrorParsingAttributeRising,
                                                          status_cat);

  xml_status = xml_element->QueryDoubleAttribute("falling", &(calibration->falling));
  if (xml_status != tinyxml2::XML_NO_ATTRIBUTE && xml_status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(CalibrationStatusCategory::ErrorParsingAttributeFalling,
                                                          status_cat);

  return status_code;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_CALIBRATION_H
