/**
 * @file collision.cpp
 * @brief Parse collision from xml string
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
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/collision.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/geometry.h>

std::vector<tesseract_scene_graph::Collision::Ptr>
tesseract_urdf::parseCollision(const tinyxml2::XMLElement* xml_element,
                               const tesseract_common::ResourceLocator& locator,
                               int version)
{
  std::vector<tesseract_scene_graph::Collision::Ptr> collisions;

  // get name
  std::string collision_name = tesseract_common::StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d collision_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      collision_origin = parseOrigin(origin, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Collision: Error parsing 'origin' element!"));
    }
  }

  // get geometry
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement("geometry");
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Collision: Error missing 'geometry' element!"));

  std::vector<tesseract_geometry::Geometry::Ptr> geometries;
  try
  {
    geometries = parseGeometry(geometry, locator, false, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Collision: Error parsing 'geometry' element!"));
  }

  if (geometries.size() == 1)
  {
    auto collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->name = collision_name;
    collision->origin = collision_origin;
    collision->geometry = geometries[0];
    collisions.push_back(collision);
  }
  else
  {
    int i = 0;
    for (const auto& g : geometries)
    {
      auto collision = std::make_shared<tesseract_scene_graph::Collision>();

      if (collision_name.empty())
        collision->name = collision_name;
      else
        collision->name = collision_name + "_" + std::to_string(i);

      collision->origin = collision_origin;
      collision->geometry = g;
      collisions.push_back(collision);
    }
  }

  return collisions;
}

tinyxml2::XMLElement*
tesseract_urdf::writeCollision(const std::shared_ptr<const tesseract_scene_graph::Collision>& collision,
                               tinyxml2::XMLDocument& doc,
                               const std::string& directory,
                               const std::string& link_name,
                               const int id = -1)
{
  if (collision == nullptr)
    std::throw_with_nested(std::runtime_error("Collision is nullptr and cannot be converted to XML"));

  tinyxml2::XMLElement* xml_element = doc.NewElement("collision");

  if (!collision->name.empty())
    xml_element->SetAttribute("name", collision->name.c_str());

  tinyxml2::XMLElement* xml_origin = writeOrigin(collision->origin, doc);
  xml_element->InsertEndChild(xml_origin);

  try
  {
    std::string filename = "collision/" + link_name + "_collision";
    if (id >= 0)
      filename += "_" + std::to_string(id);
    tinyxml2::XMLElement* xml_geometry = writeGeometry(collision->geometry, doc, directory, filename);
    xml_element->InsertEndChild(xml_geometry);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Could not write geometry for collision '" + collision->name + "'!"));
  }

  return xml_element;
}
