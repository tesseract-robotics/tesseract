/**
 * @file origin.h
 * @brief Parse origin from xml string
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_SCENE_GRAPH_URDF_PARSER_ORIGIN_H
#define TESSERACT_SCENE_GRAPH_URDF_PARSER_ORIGIN_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <vector>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/parser/urdf_parser/utils.h>

namespace tesseract_scene_graph
{

class OriginStatusCategory : public tesseract_common::StatusCategory
{
public:
  OriginStatusCategory() : name_("OriginStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessful";
      case ErrorParsingAttributeXYZ:
        return "Failed to parse origin attribute 'xyz'!";
      case ErrorParsingAttributeRPY:
        return "Failed to parse origin attribute 'rpy'!";
      case ErrorParsingAttributeXYZString:
        return "Failed to parse origin attribute 'xyz' string!";
      case ErrorParsingAttributeRPYString:
        return "Failed to parse origin attribute 'rpy' string!";
      case ErrorMissingAllAttributes:
        return "Error missing both attributes 'xyz' and 'rpy' for origin element!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorMissingAllAttributes = -1,
    ErrorParsingAttributeXYZ = -2,
    ErrorParsingAttributeRPY = -3,
    ErrorParsingAttributeXYZString = -4,
    ErrorParsingAttributeRPYString = -5
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(Eigen::Isometry3d& origin, const tinyxml2::XMLElement* xml_element)
{
  origin = Eigen::Isometry3d::Identity();
  auto status_cat = std::make_shared<OriginStatusCategory>();

  if (xml_element->Attribute("xyz") == nullptr && xml_element->Attribute("rpy") == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::ErrorMissingAllAttributes, status_cat);

  std::string xyz_string, rpy_string;
  tinyxml2::XMLError status = QueryStringAttribute(xml_element, "xyz", xyz_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::ErrorParsingAttributeXYZ, status_cat);
  }
  else if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
      return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::ErrorParsingAttributeXYZString, status_cat);

    origin.translation() = Eigen::Vector3d(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
  }

  status = QueryStringAttribute(xml_element, "rpy", rpy_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::ErrorParsingAttributeRPY, status_cat);
  }
  else if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
      return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::ErrorParsingAttributeRPYString, status_cat);

    Eigen::AngleAxisd rollAngle(std::stod(tokens[0]), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(std::stod(tokens[1]), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(std::stod(tokens[2]), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond rpy =  yawAngle * pitchAngle * rollAngle;

    origin.linear() = rpy.toRotationMatrix();
  }

  return std::make_shared<tesseract_common::StatusCode>(OriginStatusCategory::Success, status_cat);
}

}

#endif // TESSERACT_SCENE_GRAPH_URDF_PARSER_ORIGIN_H
