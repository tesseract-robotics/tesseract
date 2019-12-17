/**
 * @file visual.h
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
#ifndef TESSERACT_URDF_VISUAL_H
#define TESSERACT_URDF_VISUAL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/geometry.h>

namespace tesseract_urdf
{
class VisualStatusCategory : public tesseract_common::StatusCategory
{
public:
  VisualStatusCategory() : name_("VisualStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed 'visual' element";
      case ErrorParsingOriginElement:
        return "Error parsing visual 'origin' element!";
      case ErrorMissingMaterialElement:
        return "Error missing visual 'material' element!";
      case ErrorParsingMaterialElement:
        return "Error parsing visual 'material' element!";
      case ErrorMissingGeometryElement:
        return "Error missing visual 'geometry' element!";
      case ErrorParsingGeometryElement:
        return "Error parsing visual 'geometry' element!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorParsingOriginElement = -1,
    ErrorMissingMaterialElement = -2,
    ErrorParsingMaterialElement = -3,
    ErrorMissingGeometryElement = -4,
    ErrorParsingGeometryElement = -5,
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr
parse(std::vector<tesseract_scene_graph::Visual::Ptr>& visuals,
      const tinyxml2::XMLElement* xml_element,
      const tesseract_scene_graph::ResourceLocator::Ptr& locator,
      std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
      const int version)
{
  visuals.clear();
  auto status_cat = std::make_shared<VisualStatusCategory>();

  // get name
  std::string visual_name = StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d visual_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    auto status = parse(visual_origin, origin, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          VisualStatusCategory::ErrorParsingOriginElement, status_cat, status);
  }

  // get material
  tesseract_scene_graph::Material::Ptr visual_material = tesseract_scene_graph::DEFAULT_TESSERACT_MATERIAL;
  const tinyxml2::XMLElement* material = xml_element->FirstChildElement("material");
  if (material != nullptr)
  {
    auto status = parse(visual_material, material, available_materials, true, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          VisualStatusCategory::ErrorParsingMaterialElement, status_cat, status);
  }

  // get geometry
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement("geometry");
  if (geometry == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(VisualStatusCategory::ErrorMissingGeometryElement,
                                                          status_cat);

  std::vector<tesseract_geometry::Geometry::Ptr> geometries;
  auto status = parse(geometries, geometry, locator, true, version);
  if (!(*status))
    return std::make_shared<tesseract_common::StatusCode>(
        VisualStatusCategory::ErrorParsingGeometryElement, status_cat, status);

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

  return std::make_shared<tesseract_common::StatusCode>(VisualStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_VISUAL_H
