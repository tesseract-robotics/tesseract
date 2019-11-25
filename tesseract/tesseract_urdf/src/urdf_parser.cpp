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

#include <tesseract_urdf/urdf_parser.h>

namespace tesseract_urdf
{
tesseract_common::StatusCode::Ptr parseURDFString(tesseract_scene_graph::SceneGraph::Ptr& scene_graph,
                                                  const std::string& urdf_xml_string,
                                                  const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  scene_graph = nullptr;
  auto status_cat = std::make_shared<URDFStatusCategory>();

  tinyxml2::XMLDocument xml_doc;
  if (xml_doc.Parse(urdf_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorOpeningFile, status_cat);

  tinyxml2::XMLElement* robot = xml_doc.FirstChildElement("robot");
  if (robot == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorParsingRobotElement, status_cat);

  std::string robot_name;
  if (QueryStringAttribute(robot, "name", robot_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorAttributeName, status_cat);

  int urdf_version = 1;
  auto version_status = robot->QueryIntAttribute("version", &urdf_version);
  if (version_status != tinyxml2::XML_NO_ATTRIBUTE && version_status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorAttributeVersion, status_cat);

  status_cat = std::make_shared<URDFStatusCategory>(robot_name);

  auto sg = std::make_shared<tesseract_scene_graph::SceneGraph>();
  sg->setName(robot_name);

  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
  for (tinyxml2::XMLElement* material = robot->FirstChildElement("material"); material;
       material = material->NextSiblingElement("material"))
  {
    tesseract_scene_graph::Material::Ptr m = nullptr;
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_material;
    auto status = parse(m, material, empty_material, true, urdf_version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorParsingAvailableMaterialElement, status_cat, status);

    available_materials[m->getName()] = m;
  }

  for (tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
  {
    tesseract_scene_graph::Link::Ptr l = nullptr;
    auto status = parse(l, link, locator, available_materials, urdf_version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorParsingLinkElement, status_cat, status);

    // Check if link name is unique
    if (sg->getLink(l->getName()) != nullptr)
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorLinkNamesNotUnique, status_cat, status);

    if (!sg->addLink(tesseract_scene_graph::Link(*l)))
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorAddingLinkToSceneGraph, status_cat, status);
  }

  if (sg->getLinks().empty())
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorNoLinks, status_cat);

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("join"
                                                                                                                 "t"))
  {
    tesseract_scene_graph::Joint::Ptr j = nullptr;
    auto status = parse(j, joint, urdf_version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorParsingLinkElement, status_cat, status);

    // Check if joint name is unique
    if (sg->getJoint(j->getName()) != nullptr)
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorJointNamesNotUnique, status_cat, status);

    if (!sg->addJoint(tesseract_scene_graph::Joint(*j)))
      return std::make_shared<tesseract_common::StatusCode>(
          URDFStatusCategory::ErrorAddingJointToSceneGraph, status_cat, status);
  }

  if (sg->getJoints().empty())
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorNoJoints, status_cat);

  if (!sg->isTree())
  {
    if (sg->isAcyclic())
      return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorIsNotTree, status_cat);

    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorIsAcyclic, status_cat);
  }

  // Find root link
  for (const auto& l : sg->getLinks())
    if (sg->getInboundJoints(l->getName()).empty())
      sg->setRoot(l->getName());

  scene_graph = std::move(sg);
  return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::Success, status_cat);
}

tesseract_common::StatusCode::Ptr parseURDFFile(tesseract_scene_graph::SceneGraph::Ptr& scene_graph,
                                                const std::string& path,
                                                const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  scene_graph = nullptr;
  auto status_cat = std::make_shared<URDFStatusCategory>();

  std::ifstream ifs(path);
  if (!ifs)
    return std::make_shared<tesseract_common::StatusCode>(URDFStatusCategory::ErrorOpeningFile, status_cat);

  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
  return parseURDFString(scene_graph, urdf_xml_string, locator);
}

tesseract_scene_graph::SceneGraph::Ptr parseURDFFile(const std::string& path,
                                                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = nullptr;

  auto status = parseURDFFile(scene_graph, path, locator);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
    return nullptr;
  }

  return scene_graph;
}

tesseract_scene_graph::SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string,
                                                       const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = nullptr;

  auto status = parseURDFString(scene_graph, urdf_xml_string, locator);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
    return nullptr;
  }

  return scene_graph;
}

}  // namespace tesseract_urdf
