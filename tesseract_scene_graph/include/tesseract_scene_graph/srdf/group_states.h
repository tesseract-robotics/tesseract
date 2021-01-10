#ifndef TESSERACT_SCENE_GRAPH_SRDF_GROUP_STATES_H
#define TESSERACT_SCENE_GRAPH_SRDF_GROUP_STATES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <console_bridge/console.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kinematics_information.h>

namespace tesseract_scene_graph
{
/**
 * @brief Parse groups states from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Group states
 */
inline GroupJointStates parseGroupStates(const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const GroupNames& group_names,
                                         const tinyxml2::XMLElement* srdf_xml,
                                         const std::array<int, 3>& /*version*/)
{
  GroupJointStates group_states;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_state"); xml_element;
       xml_element = xml_element->NextSiblingElement("group_state"))
  {
    std::string group_name, state_name;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", state_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    bool found = std::find(group_names.begin(), group_names.end(), group_name) != group_names.end();
    if (!found)
    {
      CONSOLE_BRIDGE_logError("Group state '%s' specified for group '%s', but that group is not known",
                              state_name.c_str(),
                              group_name.c_str());
      continue;
    }

    auto gs = group_states.find(group_name);
    if (gs == group_states.end())
    {
      group_states[group_name] = GroupsJointStates();
      gs = group_states.find(group_name);
    }

    GroupsJointState joint_state;

    // get the joint values in the group state
    for (const tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      std::string joint_name;
      double joint_value{ 0 };
      status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      if (!scene_graph.getJoint(joint_name))
      {
        CONSOLE_BRIDGE_logError("Joint '%s' declared as part of group state '%s' is not known to the URDF",
                                joint_name.c_str(),
                                state_name.c_str());
        continue;
      }

      status = tesseract_common::QueryDoubleAttributeRequired(joint_xml, "value", joint_value);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      joint_state[joint_name] = joint_value;

      if (joint_state.empty())
        CONSOLE_BRIDGE_logError("Unable to parse joint value ('%s') for joint '%s' in group state '%s'",
                                joint_value,
                                joint_name.c_str(),
                                state_name.c_str());
    }
    gs->second[state_name] = joint_state;
  }

  return group_states;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_GROUP_STATES_H
