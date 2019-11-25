/**
 * @file collision.h
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
#ifndef TESSERACT_URDF_COLLISION_H
#define TESSERACT_URDF_COLLISION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/geometry.h>

namespace tesseract_urdf
{
class CollisionStatusCategory : public tesseract_common::StatusCategory
{
public:
  CollisionStatusCategory() : name_("CollisionStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed 'collision' element";
      case ErrorParsingOriginElement:
        return "Error parsing collision 'origin' element!";
      case ErrorMissingGeometryElement:
        return "Error missing collision 'geometry' element!";
      case ErrorParsingGeometryElement:
        return "Error parsing collision 'geometry' element!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorParsingOriginElement = -1,
    ErrorMissingGeometryElement = -2,
    ErrorParsingGeometryElement = -3
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(std::vector<tesseract_scene_graph::Collision::Ptr>& collisions,
                                               const tinyxml2::XMLElement* xml_element,
                                               const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                               const int version)
{
  collisions.clear();
  auto status_cat = std::make_shared<CollisionStatusCategory>();

  // get name
  std::string collision_name = StringAttribute(xml_element, "name", "");

  // get origin
  Eigen::Isometry3d collision_origin = Eigen::Isometry3d::Identity();
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    auto status = parse(collision_origin, origin, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          CollisionStatusCategory::ErrorParsingOriginElement, status_cat, status);
  }

  // get geometry
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement("geometry");
  if (geometry == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(CollisionStatusCategory::ErrorMissingGeometryElement,
                                                          status_cat);

  std::vector<tesseract_geometry::Geometry::Ptr> geometries;
  auto status = parse(geometries, geometry, locator, false, version);
  if (!(*status))
    return std::make_shared<tesseract_common::StatusCode>(
        CollisionStatusCategory::ErrorParsingGeometryElement, status_cat, status);

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

  return std::make_shared<tesseract_common::StatusCode>(CollisionStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_COLLISION_H
