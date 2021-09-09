/**
 * @file group_tool_center_points.cpp
 * @brief Parse group tool center points data from srdf file
 *
 * @author Levi Armstrong
 * @date March 13, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tinyxml2.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/group_tool_center_points.h>

namespace tesseract_srdf
{
/**
 * @brief Parse groups tool center points from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Group Tool Center Points
 */
GroupTCPs parseGroupTCPs(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
                         const tinyxml2::XMLElement* srdf_xml,
                         const std::array<int, 3>& /*version*/)
{
  using tesseract_common::strFormat;

  GroupTCPs group_tcps;
  for (const tinyxml2::XMLElement* xml_group_element = srdf_xml->FirstChildElement("group_tcps");
       xml_group_element != nullptr;
       xml_group_element = xml_group_element->NextSiblingElement("group_tcps"))
  {
    std::string group_name_string;
    tinyxml2::XMLError status =
        tesseract_common::QueryStringAttributeRequired(xml_group_element, "group", group_name_string);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupTCPs: Missing or failed to parse attribute 'group'!"));

    for (const tinyxml2::XMLElement* xml_element = xml_group_element->FirstChildElement("tcp"); xml_element != nullptr;
         xml_element = xml_element->NextSiblingElement("tcp"))
    {
      Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();

      if (xml_element->Attribute("name") == nullptr || xml_element->Attribute("xyz") == nullptr ||
          (xml_element->Attribute("rpy") == nullptr && xml_element->Attribute("wxyz") == nullptr))
        std::throw_with_nested(
            std::runtime_error(strFormat("GroupTCPs: Invalid tcp definition for group '%s'!", group_name_string)));

      std::string tcp_name_string;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", tcp_name_string);
      if (status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(
            std::runtime_error("GroupTCPS: Failed to parse attribute 'name' for group '" + group_name_string + "'!"));

      std::string xyz_string, rpy_string, wxyz_string;
      status = tesseract_common::QueryStringAttribute(xml_element, "xyz", xyz_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                            "attribute 'xyz'!",
                                                            tcp_name_string,
                                                            group_name_string)));

      if (status != tinyxml2::XML_NO_ATTRIBUTE)
      {
        std::vector<std::string> tokens;
        boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
          std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                              "attribute 'xyz'!",
                                                              tcp_name_string,
                                                              group_name_string)));

        double x{ 0 }, y{ 0 }, z{ 0 };
        // No need to check return values because the tokens are verified above
        tesseract_common::toNumeric<double>(tokens[0], x);
        tesseract_common::toNumeric<double>(tokens[1], y);
        tesseract_common::toNumeric<double>(tokens[2], z);

        tcp.translation() = Eigen::Vector3d(x, y, z);
      }

      if (xml_element->Attribute("wxyz") == nullptr)
      {
        status = tesseract_common::QueryStringAttribute(xml_element, "rpy", rpy_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
          std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                              "attribute 'rpy'!",
                                                              tcp_name_string,
                                                              group_name_string)));

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
            std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                                "attribute 'rpy'!",
                                                                tcp_name_string,
                                                                group_name_string)));

          double r{ 0 }, p{ 0 }, y{ 0 };
          // No need to check return values because the tokens are verified above
          tesseract_common::toNumeric<double>(tokens[0], r);
          tesseract_common::toNumeric<double>(tokens[1], p);
          tesseract_common::toNumeric<double>(tokens[2], y);

          Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

          Eigen::Quaterniond rpy = yawAngle * pitchAngle * rollAngle;

          tcp.linear() = rpy.toRotationMatrix();
        }
      }
      else
      {
        status = tesseract_common::QueryStringAttribute(xml_element, "wxyz", wxyz_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
          std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                              "attribute 'wxyz'!",
                                                              tcp_name_string,
                                                              group_name_string)));

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, wxyz_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
            std::throw_with_nested(std::runtime_error(strFormat("GroupTCPS: TCP '%s' for group '%s' failed to parse "
                                                                "attribute 'wxyz'!",
                                                                tcp_name_string,
                                                                group_name_string)));

          double qw{ 0 }, qx{ 0 }, qy{ 0 }, qz{ 0 };
          // No need to check return values because the tokens are verified above
          tesseract_common::toNumeric<double>(tokens[0], qw);
          tesseract_common::toNumeric<double>(tokens[1], qx);
          tesseract_common::toNumeric<double>(tokens[2], qy);
          tesseract_common::toNumeric<double>(tokens[3], qz);

          Eigen::Quaterniond q(qw, qx, qy, qz);
          q.normalize();

          tcp.linear() = q.toRotationMatrix();
        }
      }

      auto group_tcp = group_tcps.find(group_name_string);
      if (group_tcp == group_tcps.end())
      {
        group_tcps[group_name_string] = GroupsTCPs();
        group_tcp = group_tcps.find(group_name_string);
      }

      group_tcp->second[tcp_name_string] = tcp;
    }

    if (group_tcps.count(group_name_string) == 0)
      std::throw_with_nested(
          std::runtime_error("GroupTCPS: No tool centers points were found for group '" + group_name_string + "'!"));
  }

  return group_tcps;
}

}  // namespace tesseract_srdf
