/**
 * @file visual.cpp
 * @brief Parse visual from xml string
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

#include <tesseract_urdf/visual.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/link.h>

std::vector<tesseract_scene_graph::Visual::Ptr>
tesseract_urdf::parseVisual(const tinyxml2::XMLElement* xml_element,
                            const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                            std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
                            int version)
{
  std::vector<tesseract_scene_graph::Visual::Ptr> visuals;

  // get name
  std::string visual_name = tesseract_common::StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d visual_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      visual_origin = parseOrigin(origin, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Visual: Error parsing 'origin' element!"));
    }
  }

  // get material
  tesseract_scene_graph::Material::Ptr visual_material = tesseract_scene_graph::DEFAULT_TESSERACT_MATERIAL;
  const tinyxml2::XMLElement* material = xml_element->FirstChildElement("material");
  if (material != nullptr)
  {
    try
    {
      visual_material = parseMaterial(material, available_materials, true, version);
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

  std::vector<tesseract_geometry::Geometry::Ptr> geometries;
  try
  {
    geometries = parseGeometry(geometry, locator, true, version);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Visual: Error parsing 'geometry' element!"));
  }

  if (geometries.size() == 1)
  {
    auto visual = std::make_shared<tesseract_scene_graph::Visual>();
    visual->name = visual_name;
    visual->origin = visual_origin;
    visual->geometry = geometries[0];
    visual->material = visual_material;
    visuals.push_back(visual);
  }
  else
  {
    int i = 0;
    for (const auto& g : geometries)
    {
      auto visual = std::make_shared<tesseract_scene_graph::Visual>();

      if (visual_name.empty())
        visual->name = visual_name;
      else
        visual->name = visual_name + "_" + std::to_string(i);

      visual->origin = visual_origin;
      visual->geometry = g;
      visual->material = visual_material;
      visuals.push_back(visual);
    }
  }

  return visuals;
}
