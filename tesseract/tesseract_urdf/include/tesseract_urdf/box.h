/**
 * @file box.h
 * @brief Parse box from xml string
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
#ifndef TESSERACT_URDF_BOX_H
#define TESSERACT_URDF_BOX_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/box.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
{
class BoxStatusCategory : public tesseract_common::StatusCategory
{
public:
  BoxStatusCategory() : name_("BoxStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case ErrorAttributeSize:
        return "Missing or failed parsing box attribute size!";
      case ErrorAttributeSizeConversion:
        return "Failed converting box attribute size to vector!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeSize = -1,
    ErrorAttributeSizeConversion = -2
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_geometry::Box::Ptr& box,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int /*version*/)
{
  box = nullptr;
  auto status_cat = std::make_shared<BoxStatusCategory>();

  std::string size_string;
  if (QueryStringAttribute(xml_element, "size", size_string) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::ErrorAttributeSize, status_cat);

  std::vector<std::string> tokens;
  boost::split(tokens, size_string, boost::is_any_of(" "), boost::token_compress_on);
  if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
    return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::ErrorAttributeSizeConversion, status_cat);

  double l, w, h;
  if (!tesseract_common::toNumeric<double>(tokens[0], l) || !(l > 0))
    return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::ErrorAttributeSizeConversion, status_cat);

  if (!tesseract_common::toNumeric<double>(tokens[1], w) || !(w > 0))
    return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::ErrorAttributeSizeConversion, status_cat);

  if (!tesseract_common::toNumeric<double>(tokens[2], h) || !(h > 0))
    return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::ErrorAttributeSizeConversion, status_cat);

  box = std::make_shared<tesseract_geometry::Box>(l, w, h);
  return std::make_shared<tesseract_common::StatusCode>(BoxStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_BOX_H
