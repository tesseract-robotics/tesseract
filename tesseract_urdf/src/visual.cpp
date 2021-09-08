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
#include <tesseract_scene_graph/link.h>
#include <tesseract_common/resource_locator.h>

std::vector<tesseract_scene_graph::Visual::Ptr>
tesseract_urdf::parseVisual(const tinyxml2::XMLElement* xml_element,
                            const tesseract_common::ResourceLocator& locator,
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

tinyxml2::XMLElement* tesseract_urdf::writeVisual(const std::shared_ptr<const tesseract_scene_graph::Visual>& visual,
                                                  tinyxml2::XMLDocument& doc,
                                                  const std::string& directory,
                                                  const std::string& link_name,
                                                  const int id = -1)
{
  if (visual == nullptr)
    std::throw_with_nested(std::runtime_error("Visual is nullptr and cannot be converted to XML"));

  tinyxml2::XMLElement* xml_element = doc.NewElement("visual");

  if (!visual->name.empty())
    xml_element->SetAttribute("name", visual->name.c_str());

  tinyxml2::XMLElement* xml_origin = writeOrigin(visual->origin, doc);
  xml_element->InsertEndChild(xml_origin);

  if (visual->material != nullptr)
  {
    tinyxml2::XMLElement* xml_material = writeMaterial(visual->material, doc);
    xml_element->InsertEndChild(xml_material);
  }

  try
  {
    std::string filename = "visual/" + link_name + "_visual";
    if (id >= 0)
      filename += "_" + std::to_string(id);
    tinyxml2::XMLElement* xml_geometry = writeGeometry(visual->geometry, doc, directory, filename);
    xml_element->InsertEndChild(xml_geometry);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Could not write geometry for visual '" + visual->name + "'!"));
  }

  return xml_element;
}
