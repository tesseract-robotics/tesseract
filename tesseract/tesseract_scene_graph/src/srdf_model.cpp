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

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
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
  const char* name = srdf_xml->Attribute("name");
  if (!name)
    CONSOLE_BRIDGE_logError("No name given for the robot.");
  else
  {
    name_ = std::string(name);
    tesseract_common::trim(name_);
    if (name_ != scene_graph.getName())
      CONSOLE_BRIDGE_logError("Semantic description is not specified for the same robot as the URDF");
  }

  loadGroups(scene_graph, srdf_xml);
  loadGroupStates(scene_graph, srdf_xml);
  loadToolCenterPoints(scene_graph, srdf_xml);
  loadDisabledCollisions(scene_graph, srdf_xml);

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

  for (const auto& chain : chain_groups_)
  {
    tinyxml2::XMLElement * xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", chain.first.c_str());
    for (const auto& pair : chain.second) // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement * xml_pair = doc.NewElement("chain");
      xml_pair->SetAttribute("base_link", pair.first.c_str());
      xml_pair->SetAttribute("tip_link", pair.second.c_str());
      xml_group->InsertEndChild(xml_pair);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& joint : joint_groups_)
  {
    tinyxml2::XMLElement * xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", joint.first.c_str());
    for (const auto& joint_name : joint.second) // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement * xml_joint = doc.NewElement("joint");
      xml_joint->SetAttribute("name", joint_name.c_str());
      xml_group->InsertEndChild(xml_joint);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& link : link_groups_)
  {
    tinyxml2::XMLElement * xml_group = doc.NewElement("group");
    xml_group->SetAttribute("name", link.first.c_str());
    for (const auto& link_name : link.second) // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement * xml_link = doc.NewElement("link");
      xml_link->SetAttribute("name", link_name.c_str());
      xml_group->InsertEndChild(xml_link);
    }
    xml_root->InsertEndChild(xml_group);
  }

  for (const auto& group_state : group_states_)
  {
    for (const auto& joint_state : group_state.second) // <chain base_link="base_link" tip_link="tool0" />
    {
      tinyxml2::XMLElement * xml_group_state = doc.NewElement("group_state");
      xml_group_state->SetAttribute("name", joint_state.first.c_str());
      xml_group_state->SetAttribute("group", group_state.first.c_str());

      for (const auto& joint : joint_state.second)
      {
        tinyxml2::XMLElement * xml_joint = doc.NewElement("joint");
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
    tinyxml2::XMLElement * xml_group_tcps = doc.NewElement("group_tcps");
    xml_group_tcps->SetAttribute("group", group_tcp.first.c_str());

    for (const auto& tcp : group_tcp.second)
    {
      tinyxml2::XMLElement * xml_tcp = doc.NewElement("tcp");
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

  for (const auto& entry : acm_.getAllAllowedCollisions())
  {
    tinyxml2::XMLElement * xml_acm_entry = doc.NewElement("disable_collisions");
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

void SRDFModel::clear()
{
  name_ = "";
  chain_groups_.clear();
  joint_groups_.clear();
  link_groups_.clear();
  group_states_.clear();
  group_tcps_.clear();
  acm_.clearAllowedCollisions();
}

void SRDFModel::loadGroups(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement* srdf_xml)
{
  for (tinyxml2::XMLElement* group_xml = srdf_xml->FirstChildElement("group"); group_xml;
       group_xml = group_xml->NextSiblingElement("group"))
  {
    const char* gname = group_xml->Attribute("name");
    if (!gname)
    {
      CONSOLE_BRIDGE_logError("Group name not specified");
      continue;
    }

    std::string group_name = std::string(gname);
    tesseract_common::trim(group_name);
    std::vector<std::string> links;
    std::vector<std::string> joints;
    std::vector<std::pair<std::string, std::string>> chains;

    // get the links in the groups
    for (tinyxml2::XMLElement* link_xml = group_xml->FirstChildElement("link"); link_xml;
         link_xml = link_xml->NextSiblingElement("link"))
    {
      const char* lname = link_xml->Attribute("name");
      if (!lname)
      {
        CONSOLE_BRIDGE_logError("Link name not specified");
        continue;
      }
      std::string lname_str = boost::trim_copy(std::string(lname));
      if (!scene_graph.getLink(lname_str))
      {
        CONSOLE_BRIDGE_logError(
            "Link '%s' declared as part of group '%s' is not known to the Scene Graph", lname, gname);
        continue;
      }
      links.push_back(lname_str);
    }

    // get the joints in the groups
    for (tinyxml2::XMLElement* joint_xml = group_xml->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      const char* jname = joint_xml->Attribute("name");
      if (!jname)
      {
        CONSOLE_BRIDGE_logError("Joint name not specified");
        continue;
      }
      std::string jname_str = boost::trim_copy(std::string(jname));
      if (!scene_graph.getJoint(jname_str))
      {
        CONSOLE_BRIDGE_logError(
            "Joint '%s' declared as part of group '%s' is not known to the Scene Graph", jname, gname);
        continue;
      }
      joints.push_back(jname_str);
    }

    // get the chains in the groups
    for (tinyxml2::XMLElement* chain_xml = group_xml->FirstChildElement("chain"); chain_xml;
         chain_xml = chain_xml->NextSiblingElement("chain"))
    {
      const char* base = chain_xml->Attribute("base_link");
      const char* tip = chain_xml->Attribute("tip_link");
      if (!base)
      {
        CONSOLE_BRIDGE_logError("Base link name not specified for chain");
        continue;
      }
      if (!tip)
      {
        CONSOLE_BRIDGE_logError("Tip link name not specified for chain");
        continue;
      }
      std::string base_str = boost::trim_copy(std::string(base));
      std::string tip_str = boost::trim_copy(std::string(tip));
      if (!scene_graph.getLink(base_str))
      {
        CONSOLE_BRIDGE_logError(
            "Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph", base, gname);
        continue;
      }
      if (!scene_graph.getLink(tip_str))
      {
        CONSOLE_BRIDGE_logError(
            "Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph", tip, gname);
        continue;
      }

      chains.emplace_back(base_str, tip_str);
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
  for (tinyxml2::XMLElement* gstate_xml = srdf_xml->FirstChildElement("group_state"); gstate_xml;
       gstate_xml = gstate_xml->NextSiblingElement("group_state"))
  {
    const char* sname = gstate_xml->Attribute("name");
    const char* gname = gstate_xml->Attribute("group");
    if (!sname)
    {
      CONSOLE_BRIDGE_logError("Name of group state is not specified");
      continue;
    }
    if (!gname)
    {
      CONSOLE_BRIDGE_logError("Name of group for state '%s' is not specified", sname);
      continue;
    }

    std::string state_name = boost::trim_copy(std::string(sname));
    std::string group_name = boost::trim_copy(std::string(gname));

    bool found = std::find(group_names_.begin(), group_names_.end(), group_name) != group_names_.end();
    if (!found)
    {
      CONSOLE_BRIDGE_logError("Group state '%s' specified for group '%s', but that group is not known", sname, gname);
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
    for (tinyxml2::XMLElement* joint_xml = gstate_xml->FirstChildElement("joint"); joint_xml;
         joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      const char* jname = joint_xml->Attribute("name");
      const char* jval = joint_xml->Attribute("value");
      if (!jname)
      {
        CONSOLE_BRIDGE_logError("Joint name not specified in group state '%s'", sname);
        continue;
      }
      if (!jval)
      {
        CONSOLE_BRIDGE_logError("Joint name not specified for joint '%s' in group state '%s'", jname, sname);
        continue;
      }
      std::string jname_str = boost::trim_copy(std::string(jname));
      if (!scene_graph.getJoint(jname_str))
      {
        CONSOLE_BRIDGE_logError(
            "Joint '%s' declared as part of group state '%s' is not known to the URDF", jname, sname);
        continue;
      }
      try
      {
        std::string jval_str = std::string(jval);
        std::istringstream ss(jval_str);
        while (ss.good() && !ss.eof())
        {
          double val;
          ss >> val >> std::ws;
          joint_state[jname_str] = val;
        }
      }
      catch (const std::invalid_argument& e)
      {
        CONSOLE_BRIDGE_logError("Unable to parse joint value '%s'", jval);
      }
      catch (const std::out_of_range& e)
      {
        CONSOLE_BRIDGE_logError("Unable to parse joint value '%s' (out of range)", jval);
      }

      if (joint_state.empty())
        CONSOLE_BRIDGE_logError(
            "Unable to parse joint value ('%s') for joint '%s' in group state '%s'", jval, jname, sname);
    }
    gs->second[state_name] = joint_state;
  }
}

void SRDFModel::loadToolCenterPoints(const tesseract_scene_graph::SceneGraph& /*scene_graph*/, tinyxml2::XMLElement* srdf_xml)
{
    for (tinyxml2::XMLElement* xml_group_element = srdf_xml->FirstChildElement("group_tcps"); xml_group_element;
         xml_group_element = xml_group_element->NextSiblingElement("group_tcps"))
    {
      std::string group_name_string;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(xml_group_element, "group", group_name_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      {
        CONSOLE_BRIDGE_logError("Invalid group_tcps attribute 'group'");
        continue;
      }

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

        if (xml_element->Attribute("name") == nullptr && xml_element->Attribute("xyz") == nullptr && xml_element->Attribute("rpy") == nullptr && xml_element->Attribute("wxyz") == nullptr)
        {
          CONSOLE_BRIDGE_logError("Invalid tcp definition");
          continue;
        }
        std::string tcp_name_string;
        tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(xml_element, "name", tcp_name_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        {
          CONSOLE_BRIDGE_logError("Invalid tcp attribute 'name'");
          continue;
        }

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

void SRDFModel::loadDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph, tinyxml2::XMLElement *srdf_xml)
{
  for (tinyxml2::XMLElement* c_xml = srdf_xml->FirstChildElement("disable_collisions"); c_xml;
       c_xml = c_xml->NextSiblingElement("disable_collisions"))
  {
    const char* link1_raw = c_xml->Attribute("link1");
    const char* link2_raw = c_xml->Attribute("link2");
    if (!link1_raw || !link2_raw)
    {
      CONSOLE_BRIDGE_logError("A pair of links needs to be specified to disable collisions");
      continue;
    }

    std::string link_1 = boost::trim_copy(std::string(link1_raw));
    std::string link_2 = boost::trim_copy(std::string(link2_raw));
    std::string reason;
    if (!scene_graph.getLink(link_1))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link_1.c_str());
      continue;
    }
    if (!scene_graph.getLink(link_2))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link_2.c_str());
      continue;
    }
    const char* reason_raw = c_xml->Attribute("reason");
    if (reason_raw)
      reason = std::string(reason_raw);

    acm_.addAllowedCollision(link_1, link_2, reason);
  }
}

}  // namespace tesseract_scene_graph
