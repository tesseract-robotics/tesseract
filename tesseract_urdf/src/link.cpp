/**
 * @file link.cpp
 * @brief Parse link from xml string
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
#include <stdexcept>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/inertial.h>
#include <tesseract_urdf/visual.h>
#include <tesseract_urdf/collision.h>

tesseract_scene_graph::Link::Ptr
tesseract_urdf::parseLink(const tinyxml2::XMLElement* xml_element,
                          const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                          std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
                          int version)
{
  std::string link_name;
  if (tesseract_common::QueryStringAttribute(xml_element, "name", link_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Link: Missing or failed parsing attribute 'name'!"));

  auto l = std::make_shared<tesseract_scene_graph::Link>(link_name);

  // get inertia if it exists
  const tinyxml2::XMLElement* inertial = xml_element->FirstChildElement("inertial");
  if (inertial != nullptr)
  {
    try
    {
      l->inertial = parseInertial(inertial, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Link: Error parsing 'inertial' element for link '" + link_name + "'!"));
    }
  }

  // get visual if it exists
  for (const tinyxml2::XMLElement* visual = xml_element->FirstChildElement("visual"); visual;
       visual = visual->NextSiblingElement("visual"))
  {
    std::vector<tesseract_scene_graph::Visual::Ptr> temp_visual;
    try
    {
      temp_visual = parseVisual(visual, locator, available_materials, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Link: Error parsing 'visual' element for link '" + link_name + "'!"));
    }

    l->visual.insert(l->visual.end(), temp_visual.begin(), temp_visual.end());
  }

  // get collision if exists
  for (const tinyxml2::XMLElement* collision = xml_element->FirstChildElement("collision"); collision;
       collision = collision->NextSiblingElement("collision"))
  {
    std::vector<tesseract_scene_graph::Collision::Ptr> temp_collision;
    try
    {
      temp_collision = parseCollision(collision, locator, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Link: Error parsing 'collision' element for link '" + link_name + "'!"));
    }

    l->collision.insert(l->collision.end(), temp_collision.begin(), temp_collision.end());
  }

  return l;
}
