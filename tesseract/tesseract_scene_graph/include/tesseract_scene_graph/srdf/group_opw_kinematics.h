#ifndef TESSERACT_SCENE_GRAPH_SRDF_GROUP_OPW_KINEMATICS_H
#define TESSERACT_SCENE_GRAPH_SRDF_GROUP_OPW_KINEMATICS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <console_bridge/console.h>
#include <tinyxml2.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kinematics_information.h>

namespace tesseract_scene_graph
{
/**
 * @brief Parse group opw kinematics from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Group OPW kinematics parameters
 */
inline GroupOPWKinematics parseGroupOPWKinematics(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
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
      continue;

    auto group_opw = group_opw_kinematics.find(group_name_string);
    if (group_opw == group_opw_kinematics.end())
    {
      group_opw_kinematics[group_name_string] = OPWKinematicParameters();
      group_opw = group_opw_kinematics.find(group_name_string);
    }

    if (xml_element->Attribute("a1") == nullptr && xml_element->Attribute("a2") == nullptr &&
        xml_element->Attribute("b") == nullptr && xml_element->Attribute("c1") == nullptr &&
        xml_element->Attribute("c2") == nullptr && xml_element->Attribute("c3") == nullptr &&
        xml_element->Attribute("c4") == nullptr)
    {
      CONSOLE_BRIDGE_logError("Invalid group_opw definition, must have attributes 'a1', 'a2', 'b', 'c1', 'c2', 'c3' "
                              "and 'c4'");
      continue;
    }

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "a1", group_opw->second.a1);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "a2", group_opw->second.a2);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "b", group_opw->second.b);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c1", group_opw->second.c1);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c2", group_opw->second.c2);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c3", group_opw->second.c3);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(xml_element, "c4", group_opw->second.c4);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    std::string offsets_string;
    status = tesseract_common::QueryStringAttribute(xml_element, "offsets", offsets_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    {
      CONSOLE_BRIDGE_logError("Invalid group_opw attribute 'offsets'");
      continue;
    }

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, offsets_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 6 || !tesseract_common::isNumeric(tokens))
      {
        CONSOLE_BRIDGE_logError("Error parsing group_opw attribute 'offsets'");
        continue;
      }

      // No need to check return values because the tokens are verified above
      for (std::size_t i = 0; i < 6; ++i)
        tesseract_common::toNumeric<double>(tokens[i], group_opw->second.offsets[i]);
    }

    std::string sign_corrections_string;
    status = tesseract_common::QueryStringAttribute(xml_element, "sign_corrections", sign_corrections_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    {
      CONSOLE_BRIDGE_logError("Invalid group_opw attribute 'sign_corrections'");
      continue;
    }
    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, sign_corrections_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 6 || !tesseract_common::isNumeric(tokens))
      {
        CONSOLE_BRIDGE_logError("Error parsing group_opw attribute 'sign_corrections'");
        continue;
      }

      // No need to check return values because the tokens are verified above
      for (std::size_t i = 0; i < 6; ++i)
        tesseract_common::toNumeric<signed char>(tokens[i], group_opw->second.sign_corrections[i]);
    }
  }

  return group_opw_kinematics;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_GROUP_OPW_KINEMATICS_H
