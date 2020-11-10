#ifndef TESSERACT_SCENE_GRAPH_SRDF_GROUP_ROP_KINEMATICS_H
#define TESSERACT_SCENE_GRAPH_SRDF_GROUP_ROP_KINEMATICS_H

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
 * @brief Parse group Robot on Positioner kinematics from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Group OPW kinematics parameters
 */
inline GroupROPKinematics parseGroupROPKinematics(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
                                                  const tinyxml2::XMLElement* srdf_xml,
                                                  const std::array<int, 3>& /*version*/)
{
  GroupROPKinematics group_rop_kinematics;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_rop"); xml_element;
       xml_element = xml_element->NextSiblingElement("group_rop"))
  {
    std::string group_name_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name_string);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    // get the robot on positioner group information
    ROPKinematicParameters rop_info;
    const tinyxml2::XMLElement* manip_xml = xml_element->FirstChildElement("manipulator");
    if (manip_xml == nullptr)
    {
      CONSOLE_BRIDGE_logError("ROP Group defined, but missing manipulator element!");
      continue;
    }

    status = tesseract_common::QueryStringAttributeRequired(manip_xml, "group", rop_info.manipulator_group);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttributeRequired(manip_xml, "ik_solver", rop_info.manipulator_ik_solver);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryDoubleAttributeRequired(manip_xml, "reach", rop_info.manipulator_reach);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    const tinyxml2::XMLElement* positioner_xml = xml_element->FirstChildElement("positioner");
    if (positioner_xml == nullptr)
    {
      CONSOLE_BRIDGE_logError("ROP Group defined, but missing positioner element!");
      continue;
    }

    status = tesseract_common::QueryStringAttributeRequired(positioner_xml, "group", rop_info.positioner_group);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttribute(positioner_xml, "fk_solver", rop_info.positioner_fk_solver);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    {
      CONSOLE_BRIDGE_logInform("ROP Group, positioner element missing or failed to parse 'fk_solver' attribute!");
      continue;
    }

    // get the chains in the groups
    for (const tinyxml2::XMLElement* joint_xml = positioner_xml->FirstChildElement("joint"); positioner_xml;
         positioner_xml = positioner_xml->NextSiblingElement("joint"))
    {
      std::string joint_name;
      status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      double resolution;
      status = tesseract_common::QueryDoubleAttributeRequired(joint_xml, "resolution", resolution);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      rop_info.positioner_sample_resolution[joint_name] = resolution;
    }
    group_rop_kinematics[group_name_string] = rop_info;
  }

  return group_rop_kinematics;
}
}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_SRDF_GROUP_ROP_KINEMATICS_H
