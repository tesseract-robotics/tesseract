/**
 * @file group_states.cpp
 * @brief Parse group states data from srdf file
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

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/utils.h>
#include <tesseract_srdf/group_states.h>

namespace tesseract_srdf
{
GroupJointStates parseGroupStates(const tesseract_scene_graph::SceneGraph& scene_graph,
                                  const GroupNames& group_names,
                                  const tinyxml2::XMLElement* srdf_xml,
                                  const std::array<int, 3>& /*version*/)
{
  GroupJointStates group_states;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_state"); xml_element != nullptr;
       xml_element = xml_element->NextSiblingElement("group_state"))
  {
    std::string group_name, state_name;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupStates: Missing or failed to parse attribute 'group'!"));

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", state_name);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(
          std::runtime_error("GroupStates: Failed to parse attribute 'name' for group '" + group_name + "'!"));

    bool found = std::find(group_names.begin(), group_names.end(), group_name) != group_names.end();
    if (!found)
      std::throw_with_nested(std::runtime_error(
          tesseract_common::strFormat("GroupStates: State '%s' group '%s' does not exist!", state_name, group_name)));

    GroupsJointState joint_state;

    // get the joint values in the group state
    for (const tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml != nullptr;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      std::string joint_name;
      double joint_value{ 0 };
      status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(std::runtime_error(tesseract_common::strFormat("GroupStates: Missing or failed to parse "
                                                                              "attribute 'name' from joint element for "
                                                                              "state '%s' in group '%s'!",
                                                                              state_name,
                                                                              group_name)));

      if (!scene_graph.getJoint(joint_name))
        std::throw_with_nested(std::runtime_error(tesseract_common::strFormat("GroupStates: State '%s' for group '%s' "
                                                                              "joint name '%s' is not know to the "
                                                                              "URDF!",
                                                                              state_name,
                                                                              group_name,
                                                                              joint_name)));

      status = tesseract_common::QueryDoubleAttributeRequired(joint_xml, "value", joint_value);
      if (status != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(std::runtime_error(tesseract_common::strFormat("GroupStates: State '%s' for group '%s' "
                                                                              "joint element with joint name '%s' is "
                                                                              "missing or failed to parse attribute "
                                                                              "'value'!",
                                                                              state_name,
                                                                              group_name,
                                                                              joint_name)));

      joint_state[joint_name] = joint_value;
    }

    if (joint_state.empty())
      std::throw_with_nested(std::runtime_error(tesseract_common::strFormat(
          "GroupStates: State '%s' for group '%s' is missing joint elements!", state_name, group_name)));

    auto gs = group_states.find(group_name);
    if (gs == group_states.end())
    {
      group_states[group_name] = GroupsJointStates();
      gs = group_states.find(group_name);
    }

    gs->second[state_name] = joint_state;
  }

  return group_states;
}

}  // namespace tesseract_srdf
