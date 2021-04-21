/**
 * @file srdf_model.cpp
 * @brief Parse srdf xml
 *
 * @author Levi Armstrong, Ioan Sucan
 * @date May 12, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <unordered_map>
#include <vector>
#include <utility>
#include <console_bridge/console.h>
#include <fstream>
#include <tinyxml2.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_srdf/groups.h>
#include <tesseract_srdf/group_states.h>
#include <tesseract_srdf/group_tool_center_points.h>
#include <tesseract_srdf/group_opw_kinematics.h>
#include <tesseract_srdf/group_rep_kinematics.h>
#include <tesseract_srdf/group_rop_kinematics.h>
#include <tesseract_srdf/disabled_collisions.h>
#include <tesseract_srdf/collision_margins.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/utils.h>

namespace tesseract_srdf
{
void SRDFModel::initFile(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& filename)
{
  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while (xml_file.good())
    {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    try
    {
      initString(scene_graph, xml_string);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("SRDF: Failed to parse file '" + filename + "'!"));
    }
  }
  else
  {
    std::throw_with_nested(std::runtime_error("SRDF: Failed to open file '" + filename + "'!"));
  }
}

void SRDFModel::initString(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& xmlstring)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xmlstring.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SRDF: Failed to create XMLDocument from xml string!"));

  clear();

  const tinyxml2::XMLElement* srdf_xml = xml_doc.FirstChildElement("robot");
  if (!srdf_xml)
    std::throw_with_nested(std::runtime_error("SRDF: Missing 'robot' element in the xml file!"));

  if (!srdf_xml || std::strncmp(srdf_xml->Name(), "robot", 5) != 0)
    std::throw_with_nested(std::runtime_error("SRDF: Missing 'robot' element in the xml file!"));

  // get the robot name
  status = tesseract_common::QueryStringAttributeRequired(srdf_xml, "name", name);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SRDF: Missing or failed to parse attribute 'name'!"));

  if (name != scene_graph.getName())
    CONSOLE_BRIDGE_logError("Semantic description is not specified for the same robot as the URDF");

  std::string version_string;
  status = tesseract_common::QueryStringAttribute(srdf_xml, "version", version_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Failed to parse attribute 'version'!"));
  }
  else if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, version_string, boost::is_any_of("."), boost::token_compress_on);
    if (tokens.size() < 2 || tokens.size() > 3 || !tesseract_common::isNumeric(tokens))
      std::throw_with_nested(std::runtime_error("SRDF: Failed to parse attribute 'version'!"));

    tesseract_common::toNumeric<int>(tokens[0], version[0]);
    tesseract_common::toNumeric<int>(tokens[1], version[1]);
    if (tokens.size() == 3)
      tesseract_common::toNumeric<int>(tokens[2], version[2]);
    else
      version[2] = 0;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("SRDF Parser: The version number warning can be suppressed by adding the attribute: "
                            "version=%i.%i.%i",
                            version[0],
                            version[1],
                            version[2]);
  }

  std::tuple<GroupNames, ChainGroups, JointGroups, LinkGroups> groups_info;
  try
  {
    groups_info = parseGroups(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Error parsing srdf groups for robot '" + name + "'!"));
  }

  kinematics_information.group_names = std::get<0>(groups_info);
  kinematics_information.chain_groups = std::get<1>(groups_info);
  kinematics_information.joint_groups = std::get<2>(groups_info);
  kinematics_information.link_groups = std::get<3>(groups_info);

  try
  {
    kinematics_information.group_states =
        parseGroupStates(scene_graph, kinematics_information.group_names, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Error parsing srdf groups states for robot '" + name + "'!"));
  }

  try
  {
    kinematics_information.group_tcps = parseGroupTCPs(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf groups tool center points for robot '" + name + "'!"));
  }

  try
  {
    kinematics_information.group_opw_kinematics = parseGroupOPWKinematics(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf group opw kinematics for robot '" + name + "'!"));
  }

  try
  {
    kinematics_information.group_rop_kinematics = parseGroupROPKinematics(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Error parsing srdf group robot on a positioner kinematics for "
                                              "robot '" +
                                              name + "'!"));
  }

  try
  {
    kinematics_information.group_rep_kinematics = parseGroupREPKinematics(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Error parsing srdf group robot with external positioner "
                                              "kinematics for robot '" +
                                              name + "'!"));
  }

  try
  {
    acm = parseDisabledCollisions(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf disabled collisions for robot '" + name + "'!"));
  }

  try
  {
    collision_margin_data = parseCollisionMargins(scene_graph, srdf_xml, version);
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf collision margin data for robot '" + name + "'!"));
  }
}

bool SRDFModel::saveToFile(const std::string& file_path) const
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* xml_root = doc.NewElement("robot");
  xml_root->SetAttribute("name", name.c_str());
  xml_root->SetAttribute(
      "version",
      (std::to_string(version[0]) + "." + std::to_string(version[1]) + "." + std::to_string(version[2])).c_str());

  for (const auto& chain : kinematics_information.chain_groups)
  {
    tinyxml2::XMLElement* xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", chain.first.c_str());
    for (const auto& pair : chain.second)  // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement* xml_pair = doc.NewElement("chain");
      xml_pair->SetAttribute("base_link", pair.first.c_str());
      xml_pair->SetAttribute("tip_link", pair.second.c_str());
      xml_group->InsertEndChild(xml_pair);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& joint : kinematics_information.joint_groups)
  {
    tinyxml2::XMLElement* xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", joint.first.c_str());
    for (const auto& joint_name : joint.second)  // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement* xml_joint = doc.NewElement("joint");
      xml_joint->SetAttribute("name", joint_name.c_str());
      xml_group->InsertEndChild(xml_joint);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& link : kinematics_information.link_groups)
  {
    tinyxml2::XMLElement* xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", link.first.c_str());
    for (const auto& link_name : link.second)  // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement* xml_link = doc.NewElement("link");
      xml_link->SetAttribute("name", link_name.c_str());
      xml_group->InsertEndChild(xml_link);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& rop : kinematics_information.group_rop_kinematics)
  {
    tinyxml2::XMLElement* xml_group_rop = doc.NewElement("group_rop");
    xml_group_rop->SetAttribute("group", rop.first.c_str());
    tinyxml2::XMLElement* xml_manip = doc.NewElement("manipulator");
    xml_manip->SetAttribute("group", rop.second.manipulator_group.c_str());
    xml_manip->SetAttribute("ik_solver", rop.second.manipulator_ik_solver.c_str());
    xml_manip->SetAttribute("reach", rop.second.manipulator_reach);
    xml_group_rop->InsertEndChild(xml_manip);

    tinyxml2::XMLElement* xml_positioner = doc.NewElement("positioner");
    xml_positioner->SetAttribute("group", rop.second.positioner_group.c_str());
    xml_positioner->SetAttribute("fk_solver", rop.second.positioner_fk_solver.c_str());
    for (const auto& pair : rop.second.positioner_sample_resolution)
    {
      tinyxml2::XMLElement* xml_joint = doc.NewElement("joint");
      xml_joint->SetAttribute("name", pair.first.c_str());
      xml_joint->SetAttribute("resolution", pair.second);
      xml_positioner->InsertEndChild(xml_joint);
    }
    xml_group_rop->InsertEndChild(xml_positioner);
    xml_root->InsertEndChild(xml_group_rop);
  }

  for (const auto& rep : kinematics_information.group_rep_kinematics)
  {
    tinyxml2::XMLElement* xml_group_rep = doc.NewElement("group_rep");
    xml_group_rep->SetAttribute("group", rep.first.c_str());
    tinyxml2::XMLElement* xml_manip = doc.NewElement("manipulator");
    xml_manip->SetAttribute("group", rep.second.manipulator_group.c_str());
    xml_manip->SetAttribute("ik_solver", rep.second.manipulator_ik_solver.c_str());
    xml_manip->SetAttribute("reach", rep.second.manipulator_reach);
    xml_group_rep->InsertEndChild(xml_manip);

    tinyxml2::XMLElement* xml_positioner = doc.NewElement("positioner");
    xml_positioner->SetAttribute("group", rep.second.positioner_group.c_str());
    xml_positioner->SetAttribute("fk_solver", rep.second.positioner_fk_solver.c_str());
    for (const auto& pair : rep.second.positioner_sample_resolution)
    {
      tinyxml2::XMLElement* xml_joint = doc.NewElement("joint");
      xml_joint->SetAttribute("name", pair.first.c_str());
      xml_joint->SetAttribute("resolution", pair.second);
      xml_positioner->InsertEndChild(xml_joint);
    }
    xml_group_rep->InsertEndChild(xml_positioner);
    xml_root->InsertEndChild(xml_group_rep);
  }

  for (const auto& group_state : kinematics_information.group_states)
  {
    for (const auto& joint_state : group_state.second)  // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement* xml_group_state = doc.NewElement("group_state");
      xml_group_state->SetAttribute("name", joint_state.first.c_str());
      xml_group_state->SetAttribute("group", group_state.first.c_str());

      for (const auto& joint : joint_state.second)
      {
        tinyxml2::XMLElement* xml_joint = doc.NewElement("joint");
        xml_joint->SetAttribute("name", joint.first.c_str());
        xml_joint->SetAttribute("value", joint.second);
        xml_group_state->InsertEndChild(xml_joint);
      }
      xml_root->InsertEndChild(xml_group_state);
    }
  }

  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
  for (const auto& group_tcp : kinematics_information.group_tcps)
  {
    tinyxml2::XMLElement* xml_group_tcps = doc.NewElement("group_tcps");
    xml_group_tcps->SetAttribute("group", group_tcp.first.c_str());

    for (const auto& tcp : group_tcp.second)
    {
      tinyxml2::XMLElement* xml_tcp = doc.NewElement("tcp");
      xml_tcp->SetAttribute("name", tcp.first.c_str());

      std::stringstream xyz_string;
      xyz_string << tcp.second.translation().format(eigen_format);
      xml_tcp->SetAttribute("xyz", xyz_string.str().c_str());

      std::stringstream wxyz_string;
      Eigen::Quaterniond q(tcp.second.linear());
      wxyz_string << Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()).format(eigen_format);

      xml_tcp->SetAttribute("wxyz", wxyz_string.str().c_str());
      xml_group_tcps->InsertEndChild(xml_tcp);
    }
    xml_root->InsertEndChild(xml_group_tcps);
  }

  for (const auto& group_opw : kinematics_information.group_opw_kinematics)
  {
    tinyxml2::XMLElement* xml_group_opw = doc.NewElement("group_opw");
    xml_group_opw->SetAttribute("group", group_opw.first.c_str());
    xml_group_opw->SetAttribute("a1", group_opw.second.a1);
    xml_group_opw->SetAttribute("a2", group_opw.second.a2);
    xml_group_opw->SetAttribute("b", group_opw.second.b);
    xml_group_opw->SetAttribute("c1", group_opw.second.c1);
    xml_group_opw->SetAttribute("c2", group_opw.second.c2);
    xml_group_opw->SetAttribute("c3", group_opw.second.c3);
    xml_group_opw->SetAttribute("c4", group_opw.second.c4);

    std::string offsets_string = std::to_string(group_opw.second.offsets[0]);
    std::string sign_corrections_string = std::to_string(group_opw.second.sign_corrections[0]);

    for (std::size_t i = 1; i < 6; ++i)
    {
      offsets_string += (" " + std::to_string(group_opw.second.offsets[i]));
      sign_corrections_string += (" " + std::to_string(group_opw.second.sign_corrections[i]));
    }

    xml_group_opw->SetAttribute("offsets", offsets_string.c_str());
    xml_group_opw->SetAttribute("sign_corrections", sign_corrections_string.c_str());

    xml_root->InsertEndChild(xml_group_opw);
  }

  for (const auto& entry : acm.getAllAllowedCollisions())
  {
    tinyxml2::XMLElement* xml_acm_entry = doc.NewElement("disable_collisions");
    xml_acm_entry->SetAttribute("link1", entry.first.first.c_str());
    xml_acm_entry->SetAttribute("link2", entry.first.second.c_str());
    xml_acm_entry->SetAttribute("reason", entry.second.c_str());
    xml_root->InsertEndChild(xml_acm_entry);
  }

  if (collision_margin_data != nullptr)
  {
    tinyxml2::XMLElement* xml_cm_entry = doc.NewElement("collision_margins");
    xml_cm_entry->SetAttribute("default_margin", collision_margin_data->getDefaultCollisionMargin());
    for (const auto& entry : collision_margin_data->getPairCollisionMargins())
    {
      tinyxml2::XMLElement* xml_cm_pair_entry = doc.NewElement("pair_margin");
      xml_cm_pair_entry->SetAttribute("link1", entry.first.first.c_str());
      xml_cm_pair_entry->SetAttribute("link2", entry.first.second.c_str());
      xml_cm_pair_entry->SetAttribute("margin", entry.second);
      xml_cm_entry->InsertEndChild(xml_cm_pair_entry);

      xml_root->InsertEndChild(xml_cm_entry);
    }
  }

  doc.InsertFirstChild(xml_root);
  tinyxml2::XMLError status = doc.SaveFile(file_path.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Failed to save SRDF XML File: %s", file_path.c_str());
    return false;
  }

  return true;
}

void SRDFModel::clear()
{
  name = "undefined";
  version = { { 1, 0, 0 } };
  acm.clearAllowedCollisions();
  kinematics_information.clear();
  collision_margin_data = nullptr;
}

}  // namespace tesseract_srdf
