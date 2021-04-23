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
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/material.h>

namespace tesseract_urdf
{
tesseract_scene_graph::SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string,
                                                       const tesseract_scene_graph::ResourceLocator::Ptr& locator)
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

  auto sg = std::make_shared<tesseract_scene_graph::SceneGraph>();
  sg->setName(robot_name);

  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
  for (tinyxml2::XMLElement* material = robot->FirstChildElement("material"); material;
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

  for (tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
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

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("join"
                                                                                                                 "t"))
  {
    tesseract_scene_graph::Joint::Ptr j = nullptr;
    try
    {
      j = parseJoint(joint, urdf_version);
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

tesseract_scene_graph::SceneGraph::Ptr parseURDFFile(const std::string& path,
                                                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  std::ifstream ifs(path);
  if (!ifs)
    std::throw_with_nested(std::runtime_error("URDF: Error opening file '" + path + "'!"));

  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
  tesseract_scene_graph::SceneGraph::Ptr sg;
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

}  // namespace tesseract_urdf
