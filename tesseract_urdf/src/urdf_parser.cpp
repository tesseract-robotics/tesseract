/**
 * @file urdf_parser.h
 * @brief A urdf parser for tesseract
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <fstream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
{
tesseract_scene_graph::SceneGraph::UPtr parseURDFString(const std::string& urdf_xml_string,
                                                        const tesseract_common::ResourceLocator& locator)
{
  tinyxml2::XMLDocument xml_doc;
  if (xml_doc.Parse(urdf_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("URDF: Failed to parse urdf string!"));

  tinyxml2::XMLElement* robot = xml_doc.FirstChildElement("robot");
  if (robot == nullptr)
    std::throw_with_nested(std::runtime_error("URDF: Missing element 'robot'!"));

  std::string robot_name;
  if (tesseract_common::QueryStringAttribute(robot, "name", robot_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("URDF: Missing or failed parsing attribute 'name'!"));

  int urdf_version = 1;
  auto version_status = robot->QueryIntAttribute("version", &urdf_version);
  if (version_status != tinyxml2::XML_NO_ATTRIBUTE && version_status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(
        std::runtime_error("URDF: Failed parsing attribute 'version' for robot '" + robot_name + "'!"));

  auto sg = std::make_unique<tesseract_scene_graph::SceneGraph>();
  sg->setName(robot_name);

  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
  for (tinyxml2::XMLElement* material = robot->FirstChildElement("material"); material != nullptr;
       material = material->NextSiblingElement("material"))
  {
    tesseract_scene_graph::Material::Ptr m = nullptr;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_material;
    try
    {
      m = parseMaterial(material, empty_material, true, urdf_version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("URDF: Error parsing urdf global 'material' element for robot '" + robot_name + "'!"));
    }

    available_materials[m->getName()] = m;
  }

  for (tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link != nullptr;
       link = link->NextSiblingElement("link"))
  {
    tesseract_scene_graph::Link::Ptr l = nullptr;
    try
    {
      l = parseLink(link, locator, available_materials, urdf_version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("URDF: Error parsing 'link' element for robot '" + robot_name + "'!"));
    }
    addLink(sg, robot_name, l);
  }

  if (sg->getLinks().empty())
    std::throw_with_nested(std::runtime_error("URDF: Error no links were found for robot '" + robot_name + "'!"));

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint != nullptr;
       joint = joint->NextSiblingElement("join"
                                         "t"))
  {
    tesseract_scene_graph::Joint::Ptr urdf_joint = nullptr;
    try
    {
      urdf_joint = parseJoint(joint, urdf_version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("URDF: Error parsing 'joint' element for robot '" + robot_name + "'!"));
    }

    // Split up any joints that require it
    const auto split_joints = splitJoint(urdf_joint);

    for (const auto& l : split_joints.first)
    {
      addLink(sg, robot_name, l);
    }
    for (const auto& j : split_joints.second)
    {
      addJoint(sg, robot_name, j);
    }
  }

  if (sg->getJoints().empty())
    std::throw_with_nested(std::runtime_error("URDF: Error no joints were found for robot '" + robot_name + "'!"));

  if (!sg->isTree())
  {
    if (!sg->isAcyclic())
      std::throw_with_nested(std::runtime_error("URDF: Error, is not a tree structure and contains cycles for robot '" +
                                                robot_name + "'!"));

    std::throw_with_nested(std::runtime_error("URDF: Error, is not a tree structure for robot '" + robot_name + "'!"));
  }

  // Find root link
  for (const auto& l : sg->getLinks())
    if (sg->getInboundJoints(l->getName()).empty())
      sg->setRoot(l->getName());

  return sg;
}

tesseract_scene_graph::SceneGraph::UPtr parseURDFFile(const std::string& path,
                                                      const tesseract_common::ResourceLocator& locator)
{
  std::ifstream ifs(path);
  if (!ifs)
    std::throw_with_nested(std::runtime_error("URDF: Error opening file '" + path + "'!"));

  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
  tesseract_scene_graph::SceneGraph::UPtr sg;
  try
  {
    sg = parseURDFString(urdf_xml_string, locator);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("URDF: Error parsing file '" + path + "'!"));
  }

  return sg;
}

void writeURDFFile(const tesseract_scene_graph::SceneGraph::ConstPtr& sg,
                   const std::string& package_path,
                   const std::string& urdf_name)
{
  // Check for null input
  if (sg == nullptr)
    std::throw_with_nested(std::runtime_error("Scene Graph is nullptr and cannot be converted to URDF"));

  // If the directory does not exist, make it
  if (package_path.empty())
    std::throw_with_nested(std::runtime_error("Package path cannot be empty"));
  boost::filesystem::create_directory(boost::filesystem::path(package_path));

  // // If the collision and visual subdirectories do not exist, make them
  // boost::filesystem::create_directory(boost::filesystem::path(directory + "collision"));
  // boost::filesystem::create_directory(boost::filesystem::path(directory + "visual"));

  // Create XML Document
  tinyxml2::XMLDocument doc;

  // Add XML Declaration
  tinyxml2::XMLDeclaration* xml_declaration = doc.NewDeclaration(R"(xml version="1.0")");
  doc.InsertFirstChild(xml_declaration);

  // Assign Robot Name
  tinyxml2::XMLElement* xml_robot = doc.NewElement("robot");
  xml_robot->SetAttribute("name", sg->getName().c_str());
  // version?
  doc.InsertEndChild(xml_robot);

  // Materials were not saved anywhere at load

  // Get Links
  std::vector<std::string> link_names;
  for (const tesseract_scene_graph::Link::ConstPtr& l : sg->getLinks())
  {
    if (l == nullptr)
      std::throw_with_nested(std::runtime_error("Link is nullptr, cannot get name"));
    link_names.push_back(l->getName());
  }

  // Sort the link names using a lamda-defined comparator function.
  // The lamda takes in two strings, applies the default < operator, and returns a bool.
  std::sort(
      link_names.begin(), link_names.end(), [](const std::string& a, const std::string& b) -> bool { return a < b; });

  // Iterate through the sorted names and write the corresponding links to XML
  for (const std::string& s : link_names)
  {
    const tesseract_scene_graph::Link::ConstPtr& l = sg->getLink(s);
    try
    {
      tinyxml2::XMLElement* xml_link = writeLink(l, doc, package_path);
      xml_robot->InsertEndChild(xml_link);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write out urdf link"));
    }
  }

  // Get joints
  std::vector<std::string> joint_names;
  for (const tesseract_scene_graph::Joint::ConstPtr& j : sg->getJoints())
  {
    if (j == nullptr)
      std::throw_with_nested(std::runtime_error("Joint is nullptr, cannot get name!"));
    joint_names.push_back(j->getName());
  }

  // Sort the joint names using a lamda-defined comparator function.
  // The lamda takes in two strings, applies the default < operator, and returns a bool.
  std::sort(
      joint_names.begin(), joint_names.end(), [](const std::string& a, const std::string& b) -> bool { return a < b; });

  // Iterate through the sorted joint names and write the corresponding joints to xml
  for (const std::string& s : joint_names)
  {
    const tesseract_scene_graph::Joint::ConstPtr& j = sg->getJoint(s);
    try
    {
      tinyxml2::XMLElement* xml_joint = writeJoint(j, doc);
      xml_robot->InsertEndChild(xml_joint);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write out urdf joint"));
    }
  }

  // Check for acyclic?

  // Prepare the urdf directory
  boost::filesystem::create_directory(boost::filesystem::path(trailingSlash(package_path) + "urdf/"));

  // Write the URDF XML to a file
  std::string full_filepath;
  if (!urdf_name.empty())
    full_filepath = trailingSlash(package_path) + "urdf/" + noLeadingSlash(urdf_name) + ".urdf";
  else
    full_filepath = trailingSlash(package_path) + "urdf/" + sg->getName() + ".urdf";
  doc.SaveFile(full_filepath.c_str());
}

std::pair<std::vector<std::shared_ptr<tesseract_scene_graph::Link>>,
          std::vector<std::shared_ptr<tesseract_scene_graph::Joint>>>
splitJoint(const tesseract_scene_graph::Joint::Ptr& joint)
{
  using namespace tesseract_scene_graph;
  switch (joint->type)
  {
    case JointType::PLANAR: {
      const std::string base_name = joint->getName();
      const std::string postfix = "planar";

      auto j1 = std::make_shared<Joint>(base_name + "_x_" + postfix);
      j1->type = JointType::PRISMATIC;
      j1->axis = Eigen::Vector3d(1.0, 0.0, 0.0);  /// @todo Handle cases where joint->axis is not (0, 0, 1)
      j1->parent_link_name = joint->parent_link_name;
      j1->child_link_name = j1->getName() + "_link";
      j1->parent_to_joint_origin_transform = joint->parent_to_joint_origin_transform;
      //      j1->dynamics = joint->dynamics;
      j1->limits = std::make_shared<tesseract_scene_graph::JointLimits>(std::numeric_limits<double>::min(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max());
      //      j1->safety = joint->safety;
      //      j1->calibration = joint->calibration;
      //      j1->mimic = joint->mimic;

      auto l1 = std::make_shared<Link>(j1->getName() + "_link");

      auto j2 = std::make_shared<Joint>(base_name + "_y_" + postfix);
      j2->type = JointType::PRISMATIC;
      j2->axis = Eigen::Vector3d(0.0, 1.0, 0.0);  /// @todo Handle cases where joint->axis is not (0, 0, 1)
      j2->parent_link_name = j1->getName() + "_link";
      j2->child_link_name = j2->getName() + "_link";
      j2->parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
      //      j2->dynamics = joint->dynamics;
      j2->limits = std::make_shared<tesseract_scene_graph::JointLimits>(std::numeric_limits<double>::min(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max());
      //      j2->safety = joint->safety;
      //      j2->calibration = joint->calibration;
      //      j2->mimic = joint->mimic;

      auto l2 = std::make_shared<Link>(j2->getName() + "_link");

      auto j3 = std::make_shared<Joint>(base_name + "_yaw_" + postfix);
      j3->type = JointType::REVOLUTE;
      j3->axis = Eigen::Vector3d(0.0, 0.0, 1.0);  /// @todo Handle cases where joint->axis is not (0, 0, 1)
      j3->parent_link_name = j2->getName() + "_link";
      j3->child_link_name = joint->child_link_name;
      j3->parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
      //      j3->dynamics = joint->dynamics;
      j3->limits = std::make_shared<tesseract_scene_graph::JointLimits>(std::numeric_limits<double>::min(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max(),
                                                                        std::numeric_limits<double>::max());
      //      j3->safety = joint->safety;
      //      j3->calibration = joint->calibration;
      //      j3->mimic = joint->mimic;

      return { { l1, l2 }, { j1, j2, j3 } };
    }
    case JointType::FLOATING: {
      std::throw_with_nested(std::runtime_error("FLOATING joints are not yet supported"));
    }
    default:
      return { {}, { joint } };
  }
}

void addLink(tesseract_scene_graph::SceneGraph::UPtr& sg,
             const std::string& robot_name,
             const tesseract_scene_graph::Link::Ptr& l)
{
  // Check if link name is unique
  if (sg->getLink(l->getName()) != nullptr)
    std::throw_with_nested(std::runtime_error("URDF: Error link name '" + l->getName() + "' is not unique for robot '" +
                                              robot_name + "'!"));

  // Add link to scene graph
  if (!sg->addLink(*l))
    std::throw_with_nested(std::runtime_error("URDF: Error adding link '" + l->getName() +
                                              "' to scene graph for robot '" + robot_name + "'!"));
}

void addJoint(tesseract_scene_graph::SceneGraph::UPtr& sg,
              const std::string& robot_name,
              const tesseract_scene_graph::Joint::Ptr& j)
{
  // Check if joint name is unique
  if (sg->getJoint(j->getName()) != nullptr)
    std::throw_with_nested(std::runtime_error("URDF: Error joint name '" + j->getName() +
                                              "' is not unique for robot '" + robot_name + "'!"));

  // Add joint to scene graph
  if (!sg->addJoint(*j))
    std::throw_with_nested(std::runtime_error("URDF: Error adding joint '" + j->getName() +
                                              "' to scene graph for robot '" + robot_name + "'!"));
}

}  // namespace tesseract_urdf
