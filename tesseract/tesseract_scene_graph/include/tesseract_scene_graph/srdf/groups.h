#ifndef TESSERACT_SCENE_GRAPH_SRDF_GROUPS_H
#define TESSERACT_SCENE_GRAPH_SRDF_GROUPS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <tuple>
#include <console_bridge/console.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kinematics_information.h>

namespace tesseract_scene_graph
{
/**
 * @brief Parse groups from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return GroupNames, ChainGroups, JointGroups, LinkGroups
 */
inline std::tuple<GroupNames, ChainGroups, JointGroups, LinkGroups>
parseGroups(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tinyxml2::XMLElement* srdf_xml,
            const std::array<int, 3>& /*version*/)
{
  GroupNames group_names;
  ChainGroups chain_groups;
  LinkGroups link_groups;
  JointGroups joint_groups;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group"); xml_element;
       xml_element = xml_element->NextSiblingElement("group"))
  {
    std::string group_name;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", group_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    std::vector<std::string> links;
    std::vector<std::string> joints;
    std::vector<std::pair<std::string, std::string>> chains;

    // get the links in the groups
    for (const tinyxml2::XMLElement* link_xml = xml_element->FirstChildElement("link"); link_xml;
         link_xml = link_xml->NextSiblingElement("link"))
    {
      std::string link_name;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(link_xml, "name", link_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      if (!scene_graph.getLink(link_name))
      {
        CONSOLE_BRIDGE_logError("Link '%s' declared as part of group '%s' is not known to the Scene Graph",
                                link_name.c_str(),
                                group_name.c_str());
        continue;
      }
      links.push_back(link_name);
    }

    // get the joints in the groups
    for (const tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      std::string joint_name;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      if (!scene_graph.getJoint(joint_name))
      {
        CONSOLE_BRIDGE_logError("Joint '%s' declared as part of group '%s' is not known to the Scene Graph",
                                joint_name.c_str(),
                                group_name.c_str());
        continue;
      }
      joints.push_back(joint_name);
    }

    // get the chains in the groups
    for (const tinyxml2::XMLElement* chain_xml = xml_element->FirstChildElement("chain"); chain_xml;
         chain_xml = chain_xml->NextSiblingElement("chain"))
    {
      std::string base_link_name, tip_link_name;
      tinyxml2::XMLError status =
          tesseract_common::QueryStringAttributeRequired(chain_xml, "base_link", base_link_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      status = tesseract_common::QueryStringAttributeRequired(chain_xml, "tip_link", tip_link_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      if (!scene_graph.getLink(base_link_name))
      {
        CONSOLE_BRIDGE_logError("Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph",
                                base_link_name.c_str(),
                                group_name.c_str());
        continue;
      }
      if (!scene_graph.getLink(tip_link_name))
      {
        CONSOLE_BRIDGE_logError("Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph",
                                tip_link_name.c_str(),
                                group_name.c_str());
        continue;
      }

      chains.emplace_back(base_link_name, tip_link_name);
    }

    if (!chains.empty() && links.empty() && joints.empty())
    {
      chain_groups[group_name] = chains;
      group_names.push_back(group_name);
    }
    else if (chains.empty() && !links.empty() && joints.empty())
    {
      link_groups[group_name] = links;
      group_names.push_back(group_name);
    }
    else if (chains.empty() && links.empty() && !joints.empty())
    {
      joint_groups[group_name] = joints;
      group_names.push_back(group_name);
    }
    else
    {
      CONSOLE_BRIDGE_logWarn("Group '%s' is empty or multiple types were provided.", group_name.c_str());
    }
  }

  return std::make_tuple(group_names, chain_groups, joint_groups, link_groups);
}
}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_GROUPS_H
