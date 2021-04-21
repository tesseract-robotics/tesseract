/**
 * @file group_rop_kinematics.cpp
 * @brief Parse group robot on a positioner kinematics data from srdf file
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
#include <tesseract_srdf/group_rop_kinematics.h>

namespace tesseract_srdf
{
GroupROPKinematics parseGroupROPKinematics(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
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
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Missing or failed to parse attribute 'group'!"));

    // get the robot on positioner group information
    ROPKinematicParameters rop_info;

    status = tesseract_common::QueryStringAttribute(xml_element, "solver_name", rop_info.solver_name);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Failed to parse attribute 'solver_name' for group "
                                                "'" +
                                                group_name_string + "'!"));

    const tinyxml2::XMLElement* manip_xml = xml_element->FirstChildElement("manipulator");
    if (manip_xml == nullptr)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Missing element 'manipulator' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryStringAttributeRequired(manip_xml, "group", rop_info.manipulator_group);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'manipulator' missing or failed to parse "
                                                "attribute 'group' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryStringAttributeRequired(manip_xml, "ik_solver", rop_info.manipulator_ik_solver);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'manipulator' missing or failed to parse "
                                                "attribute 'ik_solver' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryDoubleAttributeRequired(manip_xml, "reach", rop_info.manipulator_reach);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'manipulator' missing or failed to parse "
                                                "attribute 'reach' for group '" +
                                                group_name_string + "'!"));

    const tinyxml2::XMLElement* positioner_xml = xml_element->FirstChildElement("positioner");
    if (positioner_xml == nullptr)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Missing element 'positioner' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryStringAttributeRequired(positioner_xml, "group", rop_info.positioner_group);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'positioner' missing or failed to parse "
                                                "attribute 'group' for group '" +
                                                group_name_string + "'!"));

    status = tesseract_common::QueryStringAttribute(positioner_xml, "fk_solver", rop_info.positioner_fk_solver);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'positioner' missing or failed to parse "
                                                "attribute 'fk_solver' for group '" +
                                                group_name_string + "'!"));

    // get the chains in the groups
    bool parse_joints_failed = false;
    for (const tinyxml2::XMLElement* joint_xml = positioner_xml->FirstChildElement("joint"); positioner_xml;
         positioner_xml = positioner_xml->NextSiblingElement("joint"))
    {
      if (joint_xml == nullptr)
      {
        parse_joints_failed = true;
        continue;
      }

      std::string joint_name;
      status = tesseract_common::QueryStringAttributeRequired(joint_xml, "name", joint_name);
      if (status != tinyxml2::XML_SUCCESS)
      {
        parse_joints_failed = true;
        continue;
      }

      double resolution;
      status = tesseract_common::QueryDoubleAttributeRequired(joint_xml, "resolution", resolution);
      if (status != tinyxml2::XML_SUCCESS)
      {
        parse_joints_failed = true;
        continue;
      }

      rop_info.positioner_sample_resolution[joint_name] = resolution;
    }

    if (parse_joints_failed || rop_info.positioner_sample_resolution.empty())
      std::throw_with_nested(std::runtime_error("GroupROPKinematics: Element 'positioner' missing or failed to parse "
                                                "elements 'joint' for group '" +
                                                group_name_string + "'!"));

    group_rop_kinematics[group_name_string] = rop_info;
  }

  return group_rop_kinematics;
}
}  // namespace tesseract_srdf
