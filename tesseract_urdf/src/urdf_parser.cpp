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
#include <string_view>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_common/resource_locator.h>

#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_urdf/utils.h>

static constexpr std::string_view ROBOT_ELEMENT_NAME = "robot";

namespace tesseract_urdf
{
std::unique_ptr<tesseract_scene_graph::SceneGraph> parseURDFString(const std::string& urdf_xml_string,
                                                                   const tesseract_common::ResourceLocator& locator)
{
  tinyxml2::XMLDocument xml_doc;
  if (xml_doc.Parse(urdf_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("URDF: Failed to parse urdf string!"));

  tinyxml2::XMLElement* robot = xml_doc.FirstChildElement(ROBOT_ELEMENT_NAME.data());
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

  if (urdf_version != 1)
    std::throw_with_nested(std::runtime_error("URDF: 'version' for robot '" + robot_name + "' is set to `" +
                                              std::to_string(urdf_version) +
                                              "', this is not supported, please set it to 1.0."));

  // Check for global attribute for converting meshes to convex hulls
  bool make_convex = false;
  auto make_convex_status = robot->QueryBoolAttribute("tesseract:make_convex", &make_convex);
  switch (make_convex_status)
  {
    case tinyxml2::XML_SUCCESS:
      break;
    case tinyxml2::XML_NO_ATTRIBUTE:
    {
      const std::string message = "URDF: missing boolean attribute 'tesseract:make_convex'. This attribute indicates "
                                  "whether Tesseract should globally convert all collision mesh geometries into convex "
                                  "hulls. Previous versions of Tesseract performed this conversion automatically "
                                  "(i.e., 'tesseract:make_convex=\"true\"'. If you want to perform collision checking "
                                  "with detailed meshes instead of convex hulls, set "
                                  "'tesseract:make_convex=\"false\"'. This global attribute can be overriden on a "
                                  "per-mesh basis by specifying the 'tesseract:make_convex' attribute in the 'mesh' "
                                  "element (e.g., <mesh filename=\"...\" tesseract:make_convex=\"true/false\"> .";
      std::throw_with_nested(std::runtime_error(message));
    }
    default:
      std::throw_with_nested(std::runtime_error("URDF: Failed to parse boolean attribute 'tesseract:make_convex' for "
                                                "robot '" +
                                                robot_name + "'"));
  }

  auto sg = std::make_unique<tesseract_scene_graph::SceneGraph>();
  sg->setName(robot_name);

  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
  for (tinyxml2::XMLElement* material = robot->FirstChildElement(MATERIAL_ELEMENT_NAME.data()); material != nullptr;
       material = material->NextSiblingElement(MATERIAL_ELEMENT_NAME.data()))
  {
    tesseract_scene_graph::Material::Ptr m = nullptr;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_material;
    try
    {
      m = parseMaterial(material, empty_material, true);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("URDF: Error parsing urdf global 'material' element for robot '" + robot_name + "'!"));
    }

    available_materials[m->getName()] = m;
  }

  for (tinyxml2::XMLElement* link = robot->FirstChildElement(LINK_ELEMENT_NAME.data()); link != nullptr;
       link = link->NextSiblingElement(LINK_ELEMENT_NAME.data()))
  {
    tesseract_scene_graph::Link::Ptr l = nullptr;
    try
    {
      l = parseLink(link, locator, make_convex, available_materials);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("URDF: Error parsing 'link' element for robot '" + robot_name + "'!"));
    }

    // Check if link name is unique
    if (sg->getLink(l->getName()) != nullptr)
      std::throw_with_nested(std::runtime_error("URDF: Error link name '" + l->getName() +
                                                "' is not unique for robot '" + robot_name + "'!"));

    // Add link to scene graph
    if (!sg->addLink(*l))
      std::throw_with_nested(std::runtime_error("URDF: Error adding link '" + l->getName() +
                                                "' to scene graph for robot '" + robot_name + "'!"));
  }

  if (sg->getLinks().empty())
    std::throw_with_nested(std::runtime_error("URDF: Error no links were found for robot '" + robot_name + "'!"));

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement(JOINT_ELEMENT_NAME.data()); joint != nullptr;
       joint = joint->NextSiblingElement(JOINT_ELEMENT_NAME.data()))
  {
    tesseract_scene_graph::Joint::Ptr j = nullptr;
    try
    {
      j = parseJoint(joint);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("URDF: Error parsing 'joint' element for robot '" + robot_name + "'!"));
    }

    // Check if joint name is unique
    if (sg->getJoint(j->getName()) != nullptr)
      std::throw_with_nested(std::runtime_error("URDF: Error joint name '" + j->getName() +
                                                "' is not unique for robot '" + robot_name + "'!"));

    // Add joint to scene graph
    if (!sg->addJoint(*j))
      std::throw_with_nested(std::runtime_error("URDF: Error adding joint '" + j->getName() +
                                                "' to scene graph for robot '" + robot_name + "'!"));
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

std::unique_ptr<tesseract_scene_graph::SceneGraph> parseURDFFile(const std::string& path,
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

void writeURDFFile(const std::shared_ptr<const tesseract_scene_graph::SceneGraph>& sg,
                   const std::string& package_path,
                   const std::string& urdf_name)
{
  // Check for null input
  if (sg == nullptr)
    std::throw_with_nested(std::runtime_error("Scene Graph is nullptr and cannot be converted to URDF"));

  // If the directory does not exist, make it
  if (package_path.empty())
    std::throw_with_nested(std::runtime_error("Package path cannot be empty"));
  std::filesystem::create_directory(std::filesystem::path(package_path));

  // // If the collision and visual subdirectories do not exist, make them
  // std::filesystem::create_directory(std::filesystem::path(directory + "collision"));
  // std::filesystem::create_directory(std::filesystem::path(directory + "visual"));

  // Create XML Document
  tinyxml2::XMLDocument doc;

  // Add XML Declaration
  tinyxml2::XMLDeclaration* xml_declaration = doc.NewDeclaration(R"(xml version="1.0")");
  doc.InsertFirstChild(xml_declaration);

  // Assign Robot Name
  tinyxml2::XMLElement* xml_robot = doc.NewElement(ROBOT_ELEMENT_NAME.data());
  xml_robot->SetAttribute("name", sg->getName().c_str());
  xml_robot->SetAttribute("tesseract:make_convex", false);
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
  std::filesystem::create_directory(std::filesystem::path(trailingSlash(package_path) + "urdf/"));

  // Write the URDF XML to a file
  std::string full_filepath;
  if (!urdf_name.empty())
    full_filepath = trailingSlash(package_path) + "urdf/" + noLeadingSlash(urdf_name) + ".urdf";
  else
    full_filepath = trailingSlash(package_path) + "urdf/" + sg->getName() + ".urdf";
  doc.SaveFile(full_filepath.c_str());
}

}  // namespace tesseract_urdf
