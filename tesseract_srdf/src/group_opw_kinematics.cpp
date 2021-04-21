/**
 * @file group_opw_kinematics.cpp
 * @brief Parse group opw kinematics data from srdf file
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
#include <console_bridge/console.h>
#include <tinyxml2.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/group_rep_kinematics.h>

namespace tesseract_srdf
{
GroupOPWKinematics parseGroupOPWKinematics(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
                                           const tinyxml2::XMLElement* srdf_xml,
                                           const std::array<int, 3>& /*version*/)
{
  GroupOPWKinematics group_opw_kinematics;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_opw"); xml_element;
       xml_element = xml_element->NextSiblingElement("group_opw"))
  {
    std::string group_name_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name_string);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Missing or failed to parse attribute 'group'!"));

    OPWKinematicParameters group_opw;

    if (xml_element->Attribute("a1") == nullptr || xml_element->Attribute("a2") == nullptr ||
        xml_element->Attribute("b") == nullptr || xml_element->Attribute("c1") == nullptr ||
        xml_element->Attribute("c2") == nullptr || xml_element->Attribute("c3") == nullptr ||
        xml_element->Attribute("c4") == nullptr)
    {
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Invalid group_opw definition for group '" +
                                                group_name_string +
                                                "', must have attributes 'a1', 'a2', 'b', 'c1', 'c2', 'c3' "
                                                "and 'c4'!"));
    }

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "a1", group_opw.a1);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'a1' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "a2", group_opw.a2);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'a2' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "b", group_opw.b);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'b' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c1", group_opw.c1);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'c1' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c2", group_opw.c2);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'c2' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c3", group_opw.c3);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'c3' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c4", group_opw.c4);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'c4' for group '" +
                                                group_name_string + "'!"));

    std::string offsets_string;
    status = tesseract_common::QueryStringAttribute(xml_element, "offsets", offsets_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'offsets' for group '" +
                                                group_name_string + "'!"));

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, offsets_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 6 || !tesseract_common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Invalid attribute 'offsets' for group '" +
                                                  group_name_string + "'!"));

      // No need to check return values because the tokens are verified above
      for (std::size_t i = 0; i < 6; ++i)
        tesseract_common::toNumeric<double>(tokens[i], group_opw.offsets[i]);
    }

    std::string sign_corrections_string;
    status = tesseract_common::QueryStringAttribute(xml_element, "sign_corrections", sign_corrections_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Failed to parse attribute 'sign_corrections' for "
                                                "group '" +
                                                group_name_string + "'!"));

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, sign_corrections_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 6 || !tesseract_common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Invalid attribute 'sign_corrections' for group "
                                                  "'" +
                                                  group_name_string + "'!"));

      // No need to check return values because the tokens are verified above
      bool parse_sc_failed = false;
      for (std::size_t i = 0; i < 6; ++i)
      {
        int sc{ 0 };
        tesseract_common::toNumeric<int>(tokens[i], sc);
        if (sc == 1)
          group_opw.sign_corrections[i] = 1;
        else if (sc == -1)
          group_opw.sign_corrections[i] = -1;
        else
        {
          parse_sc_failed = true;
          CONSOLE_BRIDGE_logError("OPW Group '%s' has incorrect sign correction values!", group_name_string.c_str());
          continue;
        }
      }

      if (parse_sc_failed)
        std::throw_with_nested(std::runtime_error("GroupOPWKinematics: Attribute 'sign_corrections' for group '" +
                                                  group_name_string + "' has incorrect sign correction values!"));
    }
    group_opw_kinematics[group_name_string] = group_opw;
  }

  return group_opw_kinematics;
}
}  // namespace tesseract_srdf
