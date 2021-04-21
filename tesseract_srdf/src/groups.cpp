/**
 * @file groups.cpp
 * @brief Parse groups data from srdf file
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/groups.h>

namespace tesseract_srdf
{
std::tuple<GroupNames, ChainGroups, JointGroups, LinkGroups>
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
      std::throw_with_nested(std::runtime_error("Group: Missing or failed to parse attribute 'name'!"));

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
        std::throw_with_nested(std::runtime_error("Group: '" + group_name +
                                                  "' link element is missing or failed to parse attribute 'name'!"));

      if (!scene_graph.getLink(link_name))
        std::throw_with_nested(std::runtime_error("Group: '" + group_name + "' link '" + link_name +
                                                  "' is not known to the Scene Graph!"));

      links.push_back(link_name);
    }

    // get the joints in the groups
    for (const tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      std::string joint_name;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(std::runtime_error("Group: '" + group_name +
                                                  "' joint element is missing or failed to parse attribute 'name'!"));

      if (!scene_graph.getJoint(joint_name))
        std::throw_with_nested(std::runtime_error("Group: '" + group_name + "' joint '" + joint_name +
                                                  "' is not known to the Scene Graph!"));

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
        std::throw_with_nested(std::runtime_error("Group: '" + group_name +
                                                  "' chain element is missing or failed to parse attribute "
                                                  "'base_link'!"));

      status = tesseract_common::QueryStringAttributeRequired(chain_xml, "tip_link", tip_link_name);
      if (status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(std::runtime_error("Group: '" + group_name +
                                                  "' chain element is missing or failed to parse attribute "
                                                  "'tip_link'!"));

      if (!scene_graph.getLink(base_link_name))
        std::throw_with_nested(std::runtime_error("Group: '" + group_name + "' chain element base link '" +
                                                  base_link_name + "' is not known to the Scene Graph!"));

      if (!scene_graph.getLink(tip_link_name))
        std::throw_with_nested(std::runtime_error("Group: '" + group_name + "' chain element tip link '" +
                                                  tip_link_name + "' is not known to the Scene Graph!"));

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
      std::throw_with_nested(
          std::runtime_error("Group: '" + group_name + "' is empty or multiple types were provided!"));
    }
  }

  return std::make_tuple(group_names, chain_groups, joint_groups, link_groups);
}
}  // namespace tesseract_srdf
