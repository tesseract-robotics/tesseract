/**
 * @file graph.h
 * @brief A basic scene graph using boost
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_SCENE_GRAPH_URDF_PARSER_H
#define TESSERACT_SCENE_GRAPH_URDF_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <tesseract_common/status_code.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser/joint.h>
#include <tesseract_scene_graph/parser/urdf_parser/link.h>
#include <tesseract_scene_graph/parser/urdf_parser/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser/material.h>

namespace tesseract_scene_graph
{

class URDFStatusCategory : public tesseract_common::StatusCategory
{
public:
  URDFStatusCategory(std::string desc = "") : name_("URDFStatusCategory"), desc_(desc) {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Successfully parsed urdf: '" + desc_ + "'!";
      case ErrorOpeningFile:
        return "Failed to open urdf file: '" + desc_ + "'!";
      case ErrorAttributeName:
        return "Missing or failed parsing urdf attribute 'name' for urdf '" + desc_ + "'!";
      case ErrorParsingRobotElement:
        return "Error parsing urdf 'robot' element for urdf '" + desc_ + "'!";
      case ErrorParsingAvailableMaterialElement:
        return "Error parsing urdf global 'material' element for urdf '" + desc_ + "'!";
      case ErrorParsingLinkElement:
        return "Error parsing urdf 'link' element for urdf '" + desc_ + "'!";
      case ErrorLocatingGlobalMaterial:
        return "Error parsing urdf, unable to locate global material for urdf '" + desc_ + "'!";
      case ErrorLinkNamesNotUnique:
        return "Error parsing urdf, link names are not unique for urdf '" + desc_ + "'!";
      case ErrorAddingLinkToSceneGraph:
        return "Error parsing urdf, failed to add link to scene graph for urdf '" + desc_ + "'!";
      case ErrorNoLinks:
        return "Error parsing urdf, no links were found for urdf '" + desc_ + "'!";
      case ErrorParsingJointElement:
        return "Error parsing urdf 'joint' element for urdf '" + desc_ + "'!";
      case ErrorJointNamesNotUnique:
        return "Error parsing urdf, joint names are not unique for urdf '" + desc_ + "'!";
      case ErrorAddingJointToSceneGraph:
        return "Error parsing urdf, failed to add joint to scene graph for urdf '" + desc_ + "'!";
      case ErrorNoJoints:
        return "Error parsing urdf, no joints were found for urdf '" + desc_ + "'!";
      case ErrorIsNotTree:
        return "Error parsing urdf, is not a tree structure for urdf '" + desc_ + "'!";
      case ErrorIsAcyclic:
        return "Error parsing urdf, is acyclic for urdf '" + desc_ + "'!";
      default:
        return "Invalid error code for " + name_ + " for urdf '" + desc_ + "'!";
    }
  }

  enum
  {
    Success = 0,
    ErrorOpeningFile = -1,
    ErrorParsingRobotElement = -2,
    ErrorAttributeName = -3,
    ErrorParsingAvailableMaterialElement = -4,
    ErrorParsingLinkElement = -5,
    ErrorLocatingGlobalMaterial = -6,
    ErrorLinkNamesNotUnique = -7,
    ErrorAddingLinkToSceneGraph = -8,
    ErrorNoLinks = -9,
    ErrorParsingJointElement = -10,
    ErrorJointNamesNotUnique = -11,
    ErrorAddingJointToSceneGraph = -12,
    ErrorNoJoints = -13,
    ErrorIsNotTree = -14,
    ErrorIsAcyclic = -15
  };

private:
  std::string name_;
  std::string desc_;
};

inline tesseract_common::StatusCode::Ptr parseURDFString(SceneGraph::Ptr& scene_graph,
                                                         const std::string& urdf_xml_string,
                                                         ResourceLocatorFn locator)
{
  scene_graph = nullptr;
  auto status_cat = std::make_shared<URDFStatusCategory>();

  tinyxml2::XMLDocument xml_doc;
  if (xml_doc.Parse(urdf_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorOpeningFile, status_cat);

  tinyxml2::XMLElement *robot = xml_doc.FirstChildElement("robot");
  if (robot == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorParsingRobotElement, status_cat);

  std::string robot_name;
  if (QueryStringAttribute(robot, "name", robot_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorAttributeName, status_cat);

  status_cat = std::make_shared<URDFStatusCategory>(robot_name);

  auto sg = std::make_shared<SceneGraph>();
  sg->setName(robot_name);

  std::unordered_map<std::string, Material::Ptr> available_materials;
  for (tinyxml2::XMLElement* material = robot->FirstChildElement("material"); material; material = material->NextSiblingElement("material"))
  {
    Material::Ptr m = nullptr;
    auto status = parse(m, material, std::unordered_map<std::string, Material::Ptr>());
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorParsingAvailableMaterialElement, status_cat, status);

    available_materials[m->getName()] = m;
  }

  for (tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
  {
    Link::Ptr l = nullptr;
    auto status = parse(l, link, locator, available_materials);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorParsingLinkElement, status_cat, status);

    // Check if link name is unique
    if (sg->getLink(l->getName()) != nullptr)
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorLinkNamesNotUnique, status_cat, status);

    if (!sg->addLink(Link(*l)))
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorAddingLinkToSceneGraph, status_cat, status);
  }

  if (sg->getLinks().empty())
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorNoLinks, status_cat);

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
  {
    Joint::Ptr j = nullptr;
    auto status = parse(j, joint);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorParsingLinkElement, status_cat, status);

    // Check if joint name is unique
    if (sg->getLink(j->getName()) != nullptr)
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorJointNamesNotUnique, status_cat, status);

    if (!sg->addJoint(Joint(*j)))
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorAddingJointToSceneGraph, status_cat, status);
  }

  if (sg->getJoints().empty())
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorNoJoints, status_cat);

  if (!sg->isTree())
  {
    if (sg->isAcyclic())
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorIsNotTree, status_cat);
    else
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorIsAcyclic, status_cat);
  }

  // Find root link
  for (const auto& l : sg->getLinks())
    if (sg->getInboundJoints(l->getName()).empty())
      sg->setRoot(l->getName());

  scene_graph = std::move(sg);
  return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::Success, status_cat);
}

inline tesseract_common::StatusCode::Ptr parseURDFFile(SceneGraph::Ptr& scene_graph, const std::string& path, ResourceLocatorFn locator)
{
  scene_graph = nullptr;
  auto status_cat = std::make_shared<URDFStatusCategory>();

  std::ifstream ifs(path);
  if (!ifs)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorOpeningFile, status_cat);

  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
  return parseURDFString(scene_graph, urdf_xml_string, locator);
}

inline SceneGraph::Ptr parseURDFFile(const std::string& path, ResourceLocatorFn locator)
{
  SceneGraph::Ptr scene_graph = nullptr;

  auto status = parseURDFFile(scene_graph, path, locator);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
    return nullptr;
  }

  return scene_graph;
}

inline SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string, ResourceLocatorFn locator)
{
  SceneGraph::Ptr scene_graph = nullptr;

  auto status = parseURDFString(scene_graph, urdf_xml_string, locator);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
    return nullptr;
  }

  return scene_graph;
}


}  // namespace tesseract_scene_graph

#endif
