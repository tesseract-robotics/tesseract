/**
 * @file collision.cpp
 * @brief Parse collision from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
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

#include <Eigen/Geometry>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometry.h>
#include <tesseract_urdf/collision.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_urdf/origin.h>

namespace tesseract_urdf
{
tesseract_scene_graph::Collision::Ptr parseCollision(const tinyxml2::XMLElement* xml_element,
                                                     const tesseract_common::ResourceLocator& locator,
                                                     bool make_convex_meshes)
{
  // get name
  std::string collision_name = tesseract_common::StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d collision_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      collision_origin = parseOrigin(origin);
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

  tesseract_geometry::Geometry::Ptr geom;
  try
  {
    geom = parseGeometry(geometry, locator, false, make_convex_meshes);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Collision: Error parsing 'geometry' element!"));
  }

  auto collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = collision_name;
  collision->origin = collision_origin;
  collision->geometry = geom;

  return collision;
}

tinyxml2::XMLElement* writeCollision(const std::shared_ptr<const tesseract_scene_graph::Collision>& collision,
                                     tinyxml2::XMLDocument& doc,
                                     const std::string& package_path,
                                     const std::string& link_name,
                                     const int id = -1)
{
  if (collision == nullptr)
    std::throw_with_nested(std::runtime_error("Collision is nullptr and cannot be converted to XML"));

  tinyxml2::XMLElement* xml_element = doc.NewElement(COLLISION_ELEMENT_NAME.data());

  if (!collision->name.empty())
    xml_element->SetAttribute("name", collision->name.c_str());

  if (!collision->origin.matrix().isIdentity(std::numeric_limits<double>::epsilon()))
  {
    tinyxml2::XMLElement* xml_origin = writeOrigin(collision->origin, doc);
    xml_element->InsertEndChild(xml_origin);
  }

  // Construct filename, without extension (could be .ply or .bt)
  std::string filename = link_name;
  if (!collision->name.empty())
    filename = filename + "_" + collision->name;
  else
    filename = filename + "_collision";

  // If a package path was specified, save in a collision sub-directory
  if (!package_path.empty())
    filename = "collision/" + filename;

  // If there is more than one collision object for this link, append the id
  if (id >= 0)
    filename = filename + "_" + std::to_string(id);

  try
  {
    tinyxml2::XMLElement* xml_geometry = writeGeometry(collision->geometry, doc, package_path, filename);
    xml_element->InsertEndChild(xml_geometry);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Could not write geometry for collision '" + collision->name + "'!"));
  }

  return xml_element;
}

}  // namespace tesseract_urdf
