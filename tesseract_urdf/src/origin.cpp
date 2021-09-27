/**
 * @file origin.cpp
 * @brief Parse origin from xml string
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/origin.h>

Eigen::Isometry3d tesseract_urdf::parseOrigin(const tinyxml2::XMLElement* xml_element, int /*version*/)
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();

  if (xml_element->Attribute("xyz") == nullptr && xml_element->Attribute("rpy") == nullptr &&
      xml_element->Attribute("wxyz") == nullptr)
    std::throw_with_nested(std::runtime_error("Origin: Error missing required attributes 'xyz' and 'rpy' and/or 'wxyz' "
                                              "for origin element!"));

  std::string xyz_string, rpy_string, wxyz_string;
  tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(xml_element, "xyz", xyz_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'xyz'!"));

  if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
      std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'xyz' string!"));

    double x{ 0 }, y{ 0 }, z{ 0 };
    // No need to check return values because the tokens are verified above
    tesseract_common::toNumeric<double>(tokens[0], x);
    tesseract_common::toNumeric<double>(tokens[1], y);
    tesseract_common::toNumeric<double>(tokens[2], z);

    origin.translation() = Eigen::Vector3d(x, y, z);
  }

  if (xml_element->Attribute("wxyz") == nullptr)
  {
    status = tesseract_common::QueryStringAttribute(xml_element, "rpy", rpy_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'rpy'!"));

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'rpy' string!"));

      double r{ 0 }, p{ 0 }, y{ 0 };
      // No need to check return values because the tokens are verified above
      tesseract_common::toNumeric<double>(tokens[0], r);
      tesseract_common::toNumeric<double>(tokens[1], p);
      tesseract_common::toNumeric<double>(tokens[2], y);

      Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond rpy = yawAngle * pitchAngle * rollAngle;

      origin.linear() = rpy.toRotationMatrix();
    }
  }
  else
  {
    status = tesseract_common::QueryStringAttribute(xml_element, "wxyz", wxyz_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'wxyz'!"));

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, wxyz_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("Origin: Failed to parse attribute 'wxyz' string!"));

      double qw{ 0 }, qx{ 0 }, qy{ 0 }, qz{ 0 };
      // No need to check return values because the tokens are verified above
      tesseract_common::toNumeric<double>(tokens[0], qw);
      tesseract_common::toNumeric<double>(tokens[1], qx);
      tesseract_common::toNumeric<double>(tokens[2], qy);
      tesseract_common::toNumeric<double>(tokens[3], qz);

      Eigen::Quaterniond q(qw, qx, qy, qz);
      q.normalize();

      origin.linear() = q.toRotationMatrix();
    }
  }
  return origin;
}

tinyxml2::XMLElement* tesseract_urdf::writeOrigin(const Eigen::Isometry3d& origin, tinyxml2::XMLDocument& doc)
{
  tinyxml2::XMLElement* xml_element = doc.NewElement("origin");

  // Format and write the translation
  std::string xyz_string = std::to_string(origin.translation().x()) + " " + std::to_string(origin.translation().y()) +
                           " " + std::to_string(origin.translation().z());
  xml_element->SetAttribute("xyz", xyz_string.c_str());

  // Extract, format, and write the rotation
  Eigen::Quaterniond q(origin.linear());
  std::string wxyz_string =
      std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
  xml_element->SetAttribute("wxyz", wxyz_string.c_str());

  return xml_element;
}
