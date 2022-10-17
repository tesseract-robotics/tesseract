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
#include <boost/serialization/array.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>
#include <unordered_map>
#include <vector>
#include <utility>
#include <console_bridge/console.h>
#include <fstream>
#include <tinyxml2.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_srdf/groups.h>
#include <tesseract_srdf/group_states.h>
#include <tesseract_srdf/group_tool_center_points.h>
#include <tesseract_srdf/disabled_collisions.h>
#include <tesseract_srdf/collision_margins.h>
#include <tesseract_srdf/configs.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/eigen_serialization.h>

namespace tesseract_srdf
{
void SRDFModel::initFile(const tesseract_scene_graph::SceneGraph& scene_graph,
                         const std::string& filename,
                         const tesseract_common::ResourceLocator& locator)
{
  // get the entire file
  tesseract_common::Resource::Ptr resource = locator.locateResource(filename);
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open() && resource)
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
      initString(scene_graph, xml_string, *resource);
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

void SRDFModel::initString(const tesseract_scene_graph::SceneGraph& scene_graph,
                           const std::string& xmlstring,
                           const tesseract_common::ResourceLocator& locator)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xmlstring.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SRDF: Failed to create XMLDocument from xml string!"));

  clear();

  const tinyxml2::XMLElement* srdf_xml = xml_doc.FirstChildElement("robot");
  if (srdf_xml == nullptr)
    std::throw_with_nested(std::runtime_error("SRDF: Missing 'robot' element in the xml file!"));

  if (srdf_xml == nullptr || std::strncmp(srdf_xml->Name(), "robot", 5) != 0)
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
    for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("kinematics_plugin_config");
         xml_element != nullptr;
         xml_element = xml_element->NextSiblingElement("kinematics_plugin_config"))
    {
      tesseract_common::KinematicsPluginInfo info = parseKinematicsPluginConfig(locator, xml_element, version);
      kinematics_information.kinematics_plugin_info.insert(info);
    }
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf kinematics plugin config for robot '" + name + "'!"));
  }

  try
  {
    for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("calibration_config");
         xml_element != nullptr;
         xml_element = xml_element->NextSiblingElement("calibration_config"))
    {
      tesseract_common::CalibrationInfo info = parseCalibrationConfig(scene_graph, locator, xml_element, version);
      calibration_info.insert(info);
    }
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("SRDF: Error parsing srdf calibration config for robot '" + name + "'!"));
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

  try
  {
    for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("contact_managers_plugin_config");
         xml_element != nullptr;
         xml_element = xml_element->NextSiblingElement("contact_managers_plugin_config"))
    {
      tesseract_common::ContactManagersPluginInfo info =
          parseContactManagersPluginConfig(locator, xml_element, version);
      contact_managers_plugin_info.insert(info);
    }
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SRDF: Error parsing srdf contact managers plugin config for robot '" + name + "'!"));
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

  if (!kinematics_information.kinematics_plugin_info.empty())
  {
    tesseract_common::fs::path p(file_path);
    std::ofstream fout(p.parent_path().append("kinematics_plugin_config.yaml").string());
    YAML::Node config;
    config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY] = kinematics_information.kinematics_plugin_info;
    fout << config;
    tinyxml2::XMLElement* xml_kin_plugin_entry = doc.NewElement("kinematics_plugin_config");
    xml_kin_plugin_entry->SetAttribute("filename", "kinematics_plugin_config.yaml");
    xml_root->InsertEndChild(xml_kin_plugin_entry);
  }

  if (!contact_managers_plugin_info.empty())
  {
    tesseract_common::fs::path p(file_path);
    std::ofstream fout(p.parent_path().append("contact_managers_plugin_config.yaml").string());
    YAML::Node config;
    config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY] = contact_managers_plugin_info;
    fout << config;
    tinyxml2::XMLElement* xml_kin_plugin_entry = doc.NewElement("contact_managers_plugin_config");
    xml_kin_plugin_entry->SetAttribute("filename", "contact_managers_plugin_config.yaml");
    xml_root->InsertEndChild(xml_kin_plugin_entry);
  }

  if (!calibration_info.empty())
  {
    tesseract_common::fs::path p(file_path);
    std::ofstream fout(p.parent_path().append("calibration_config.yaml").string());
    YAML::Node config;
    config[tesseract_common::CalibrationInfo::CONFIG_KEY] = calibration_info;
    fout << config;
    tinyxml2::XMLElement* xml_cal_info_entry = doc.NewElement("calibration_config");
    xml_cal_info_entry->SetAttribute("filename", "calibration_config.yaml");
    xml_root->InsertEndChild(xml_cal_info_entry);
  }

  // Write the ACM
  const auto allowed_collision_entries = acm.getAllAllowedCollisions();
  auto acm_keys = getAlphabeticalACMKeys(allowed_collision_entries);
  for (const auto& key : acm_keys)
  {
    tinyxml2::XMLElement* xml_acm_entry = doc.NewElement("disable_collisions");
    xml_acm_entry->SetAttribute("link1", key.get().first.c_str());
    xml_acm_entry->SetAttribute("link2", key.get().second.c_str());
    xml_acm_entry->SetAttribute("reason", allowed_collision_entries.at(key.get()).c_str());
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
  kinematics_information.clear();
  contact_managers_plugin_info.clear();
  acm.clearAllowedCollisions();
  collision_margin_data = nullptr;
}

bool SRDFModel::operator==(const SRDFModel& rhs) const
{
  bool equal = true;
  equal &= name == rhs.name;
  equal &= tesseract_common::isIdenticalArray<int, 3>(version, rhs.version);
  equal &= kinematics_information == rhs.kinematics_information;
  equal &= contact_managers_plugin_info == rhs.contact_managers_plugin_info;
  equal &= acm == rhs.acm;
  equal &= tesseract_common::pointersEqual(collision_margin_data, rhs.collision_margin_data);
  equal &= calibration_info == rhs.calibration_info;

  return equal;
}
bool SRDFModel::operator!=(const SRDFModel& rhs) const { return !operator==(rhs); }

template <class Archive>
void SRDFModel::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(name);
  ar& BOOST_SERIALIZATION_NVP(version);
  ar& BOOST_SERIALIZATION_NVP(kinematics_information);
  ar& BOOST_SERIALIZATION_NVP(contact_managers_plugin_info);
  ar& BOOST_SERIALIZATION_NVP(acm);
  ar& BOOST_SERIALIZATION_NVP(collision_margin_data);
  ar& BOOST_SERIALIZATION_NVP(calibration_info);
}

}  // namespace tesseract_srdf

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_srdf::SRDFModel)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_srdf::SRDFModel)
