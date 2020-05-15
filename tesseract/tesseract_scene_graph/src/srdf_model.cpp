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
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/srdf_model.h>
#include <tesseract_common/utils.h>

namespace tesseract_scene_graph
{
bool SRDFModel::initXml(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml)
{
  clear();
  if (!srdf_xml || std::strncmp(srdf_xml->Name(), "robot", 5) != 0)
  {
    CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
    return false;
  }

  // get the robot name
  tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(srdf_xml, "name", name_);
  if (status != tinyxml2::XML_SUCCESS)
    return false;

  if (name_ != scene_graph.getName())
    CONSOLE_BRIDGE_logError("Semantic description is not specified for the same robot as the URDF");

  std::string version_string;
  status = tesseract_common::QueryStringAttribute(srdf_xml, "version", version_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Error parsing robot attribute 'version'");
    return false;
  }
  else if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, version_string, boost::is_any_of("."), boost::token_compress_on);
    if (tokens.size() < 2 || tokens.size() > 3 || !tesseract_common::isNumeric(tokens))
    {
      CONSOLE_BRIDGE_logError("Error parsing robot attribute 'version'");
      return false;
    }

    tesseract_common::toNumeric<int>(tokens[0], version_[0]);
    tesseract_common::toNumeric<int>(tokens[1], version_[1]);
    if (tokens.size() == 3)
      tesseract_common::toNumeric<int>(tokens[2], version_[2]);
    else
      version_[2] = 0;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("No version number was provided so latest parser will be used.");
  }

  loadGroups(scene_graph, srdf_xml);
  loadGroupStates(scene_graph, srdf_xml);
  loadToolCenterPoints(scene_graph, srdf_xml);
  loadDisabledCollisions(scene_graph, srdf_xml);
  loadGroupOPWKinematics(scene_graph, srdf_xml);

  return true;
}

bool SRDFModel::initXml(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLDocument* srdf_xml)
{
  tinyxml2::XMLElement* robot_xml = srdf_xml ? srdf_xml->FirstChildElement("robot") : nullptr;
  if (!robot_xml)
  {
    CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
    return false;
  }
  return initXml(scene_graph, robot_xml);
}

bool SRDFModel::initFile(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& filename)
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
    return initString(scene_graph, xml_string);
  }

  CONSOLE_BRIDGE_logError("Could not open file [%s] for parsing.", filename.c_str());
  return false;
}

bool SRDFModel::initString(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& xmlstring)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xmlstring.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Could not parse the SRDF XML File. %s", xml_doc.ErrorStr());
    return false;
  }

  return initXml(scene_graph, &xml_doc);
}

bool SRDFModel::saveToFile(const std::string& file_path) const
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* xml_root = doc.NewElement("robot");
  xml_root->SetAttribute("name", name_.c_str());
  xml_root->SetAttribute(
      "version",
      (std::to_string(version_[0]) + "." + std::to_string(version_[1]) + "." + std::to_string(version_[2])).c_str());

  for (const auto& chain : chain_groups_)
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

  for (const auto& joint : joint_groups_)
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

  for (const auto& link : link_groups_)
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

  for (const auto& group_state : group_states_)
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

  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, 0, " ", " ");
  for (const auto& group_tcp : group_tcps_)
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

  for (const auto& group_opw : group_opw_kinematics_)
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

  for (const auto& entry : acm_.getAllAllowedCollisions())
  {
    tinyxml2::XMLElement* xml_acm_entry = doc.NewElement("disable_collisions");
    xml_acm_entry->SetAttribute("link1", entry.first.first.c_str());
    xml_acm_entry->SetAttribute("link2", entry.first.second.c_str());
    xml_acm_entry->SetAttribute("reason", entry.second.c_str());
    xml_root->InsertEndChild(xml_acm_entry);
  }

  doc.InsertFirstChild(xml_root);
  tinyxml2::XMLError status = doc.SaveFile(file_path.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Failed to save SRDF XML File: %s, Error: &s", file_path.c_str(), doc.ErrorStr());
    return false;
  }

  return true;
}

const std::string& SRDFModel::getName() const { return name_; }
std::string& SRDFModel::getName() { return name_; }

const AllowedCollisionMatrix& SRDFModel::getAllowedCollisionMatrix() const { return acm_; }
AllowedCollisionMatrix& SRDFModel::getAllowedCollisionMatrix() { return acm_; };

const SRDFModel::ChainGroups& SRDFModel::getChainGroups() const { return chain_groups_; }
SRDFModel::ChainGroups& SRDFModel::getChainGroups() { return chain_groups_; }

const SRDFModel::JointGroups& SRDFModel::getJointGroups() const { return joint_groups_; }
SRDFModel::JointGroups& SRDFModel::getJointGroups() { return joint_groups_; }

const SRDFModel::LinkGroups& SRDFModel::getLinkGroups() const { return link_groups_; }
SRDFModel::LinkGroups& SRDFModel::getLinkGroups() { return link_groups_; }

const SRDFModel::GroupTCPs& SRDFModel::getGroupTCPs() const { return group_tcps_; }
SRDFModel::GroupTCPs& SRDFModel::getGroupTCPs() { return group_tcps_; }

const SRDFModel::GroupStates& SRDFModel::getGroupStates() const { return group_states_; }
SRDFModel::GroupStates& SRDFModel::getGroupStates() { return group_states_; }

const SRDFModel::GroupOPWKinematics& SRDFModel::getGroupOPWKinematics() const { return group_opw_kinematics_; }
SRDFModel::GroupOPWKinematics& SRDFModel::getGroupOPWKinematics() { return group_opw_kinematics_; }

void SRDFModel::clear()
{
  name_ = "";
  chain_groups_.clear();
  joint_groups_.clear();
  link_groups_.clear();
  group_states_.clear();
  group_tcps_.clear();
  acm_.clearAllowedCollisions();
  group_opw_kinematics_.clear();
}

void SRDFModel::loadGroups(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group"); xml_element;
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
    for (tinyxml2::XMLElement* link_xml = xml_element->FirstChildElement("link"); link_xml;
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
    for (tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml;
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
    for (tinyxml2::XMLElement* chain_xml = xml_element->FirstChildElement("chain"); chain_xml;
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
      chain_groups_[group_name] = chains;
      group_names_.push_back(group_name);
    }
    else if (chains.empty() && !links.empty() && joints.empty())
    {
      link_groups_[group_name] = links;
      group_names_.push_back(group_name);
    }
    else if (chains.empty() && links.empty() && !joints.empty())
    {
      joint_groups_[group_name] = joints;
      group_names_.push_back(group_name);
    }
    else
    {
      CONSOLE_BRIDGE_logWarn("Group '%s' is empty or multiple types were provided.", group_name.c_str());
    }
  }
}

void SRDFModel::loadGroupStates(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_state"); xml_element;
       xml_element = xml_element->NextSiblingElement("group_state"))
  {
    std::string group_name, state_name;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", state_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    bool found = std::find(group_names_.begin(), group_names_.end(), group_name) != group_names_.end();
    if (!found)
    {
      CONSOLE_BRIDGE_logError("Group state '%s' specified for group '%s', but that group is not known",
                              state_name.c_str(),
                              group_name.c_str());
      continue;
    }

    auto gs = group_states_.find(group_name);
    if (gs == group_states_.end())
    {
      group_states_[group_name] = JointStates();
      gs = group_states_.find(group_name);
    }

    JointState joint_state;

    // get the joint values in the group state
    for (tinyxml2::XMLElement* joint_xml = xml_element->FirstChildElement("joint"); joint_xml;
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
}

void SRDFModel::loadToolCenterPoints(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
                                     tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* xml_group_element = srdf_xml->FirstChildElement("group_tcps"); xml_group_element;
       xml_group_element = xml_group_element->NextSiblingElement("group_tcps"))
  {
    std::string group_name_string;
    tinyxml2::XMLError status =
        tesseract_common::QueryStringAttributeRequired(xml_group_element, "group", group_name_string);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    auto group_tcp = group_tcps_.find(group_name_string);
    if (group_tcp == group_tcps_.end())
    {
      group_tcps_[group_name_string] = TCPs();
      group_tcp = group_tcps_.find(group_name_string);
    }

    for (tinyxml2::XMLElement* xml_element = xml_group_element->FirstChildElement("tcp"); xml_element;
         xml_element = xml_element->NextSiblingElement("tcp"))
    {
      Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();

      if (xml_element->Attribute("name") == nullptr && xml_element->Attribute("xyz") == nullptr &&
          xml_element->Attribute("rpy") == nullptr && xml_element->Attribute("wxyz") == nullptr)
      {
        CONSOLE_BRIDGE_logError("Invalid tcp definition");
        continue;
      }
      std::string tcp_name_string;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "name", tcp_name_string);
      if (status != tinyxml2::XML_SUCCESS)
        continue;

      std::string xyz_string, rpy_string, wxyz_string;
      status = tesseract_common::QueryStringAttribute(xml_element, "xyz", xyz_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      {
        CONSOLE_BRIDGE_logError("Invalid tcp attribute 'xyz'");
        continue;
      }

      if (status != tinyxml2::XML_NO_ATTRIBUTE)
      {
        std::vector<std::string> tokens;
        boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
        {
          CONSOLE_BRIDGE_logError("Error parsing tcp attribute 'xyz'");
          continue;
        }

        double x, y, z;
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
        {
          CONSOLE_BRIDGE_logError("Invalid tcp attribute 'rpy'");
          continue;
        }

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
          {
            CONSOLE_BRIDGE_logError("Error parsing tcp attribute 'rpy'");
            continue;
          }

          double r, p, y;
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
        {
          CONSOLE_BRIDGE_logError("Invalid tcp attribute 'wxyz'");
          continue;
        }

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, wxyz_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
          {
            CONSOLE_BRIDGE_logError("Error parsing tcp attribute 'wxyz'");
            continue;
          }

          double qw, qx, qy, qz;
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

      group_tcp->second[tcp_name_string] = tcp;
    }
  }
}

void SRDFModel::loadGroupOPWKinematics(const tesseract_scene_graph::SceneGraph& /*scene_graph*/,
                                       tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("group_opw"); xml_element;
       xml_element = xml_element->NextSiblingElement("group_opw"))
  {
    std::string group_name_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "group", group_name_string);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    auto group_opw = group_opw_kinematics_.find(group_name_string);
    if (group_opw == group_opw_kinematics_.end())
    {
      group_opw_kinematics_[group_name_string] = OPWKinematicParameters();
      group_opw = group_opw_kinematics_.find(group_name_string);
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
}

void SRDFModel::loadDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph,
                                       tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("disable_collisions"); xml_element;
       xml_element = xml_element->NextSiblingElement("disable_collisions"))
  {
    std::string link1_name, link2_name, reason;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "link1", link1_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "link2", link2_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    if (!scene_graph.getLink(link1_name))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link1_name.c_str());
      continue;
    }
    if (!scene_graph.getLink(link2_name))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link2_name.c_str());
      continue;
    }

    status = tesseract_common::QueryStringAttribute(xml_element, "reason", reason);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    {
      CONSOLE_BRIDGE_logError("Invalid disable_collisions attribute 'reason'");
      continue;
    }

    acm_.addAllowedCollision(link1_name, link2_name, reason);
  }
}

}  // namespace tesseract_scene_graph
