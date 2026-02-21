/**
 * @file visual.cpp
 * @brief Parse visual from xml string
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

#include <tesseract_urdf/visual.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometry.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract::urdf
{
tesseract::scene_graph::Visual::Ptr
parseVisual(const tinyxml2::XMLElement* xml_element,
            const tesseract::common::ResourceLocator& locator,
            std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr>& available_materials)
{
  // get name
  std::string visual_name = tesseract::common::StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d visual_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      visual_origin = parseOrigin(origin);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Visual: Error parsing 'origin' element!"));
    }
  }

  // get material
  tesseract::scene_graph::Material::Ptr visual_material = tesseract::scene_graph::Material::getDefaultMaterial();
  const tinyxml2::XMLElement* material = xml_element->FirstChildElement("material");
  if (material != nullptr)
  {
    try
    {
      visual_material = parseMaterial(material, available_materials, true);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Visual: Error parsing 'material' element!"));
    }
  }

  // get geometry
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement("geometry");
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Visual: Error missing 'geometry' element!"));

  tesseract::geometry::Geometry::Ptr geom;
  try
  {
    // Set `make_convex_meshes` argument to false for visual geometry
    // Note: mesh elements can still override
    geom = parseGeometry(geometry, locator, true, false);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Visual: Error parsing 'geometry' element!"));
  }

  auto visual = std::make_shared<tesseract::scene_graph::Visual>();
  visual->name = visual_name;
  visual->origin = visual_origin;
  visual->geometry = geom;
  visual->material = visual_material;

  return visual;
}

tinyxml2::XMLElement* writeVisual(const std::shared_ptr<const tesseract::scene_graph::Visual>& visual,
                                  tinyxml2::XMLDocument& doc,
                                  const std::string& package_path,
                                  const std::string& link_name,
                                  const int id = -1)
{
  if (visual == nullptr)
    std::throw_with_nested(std::runtime_error("Visual is nullptr and cannot be converted to XML"));

  tinyxml2::XMLElement* xml_element = doc.NewElement(VISUAL_ELEMENT_NAME.data());

  if (!visual->name.empty())
    xml_element->SetAttribute("name", visual->name.c_str());

  if (!visual->origin.matrix().isIdentity(std::numeric_limits<double>::epsilon()))
  {
    tinyxml2::XMLElement* xml_origin = writeOrigin(visual->origin, doc);
    xml_element->InsertEndChild(xml_origin);
  }

  if (visual->material != nullptr)
  {
    tinyxml2::XMLElement* xml_material = writeMaterial(visual->material, doc);
    xml_element->InsertEndChild(xml_material);
  }

  // Construct Filename, without extension (could be .ply or .bt)
  std::string filename = link_name;
  if (!visual->name.empty())
    filename = filename + "_" + visual->name;
  else
    filename = filename + "_visual";

  // If a package path was specified, save in a visual sub-directory
  if (!package_path.empty())
    filename = "visual/" + filename;

  // If there is more than one visual object for this link, append the id
  if (id >= 0)
    filename = filename + "_" + std::to_string(id);

  try
  {
    std::string filename = "visual/" + link_name + "_visual";
    if (id >= 0)
      filename += "_" + std::to_string(id);
    tinyxml2::XMLElement* xml_geometry = writeGeometry(visual->geometry, doc, package_path, filename);
    xml_element->InsertEndChild(xml_geometry);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Could not write geometry for visual '" + visual->name + "'!"));
  }

  return xml_element;
}

}  // namespace tesseract::urdf
