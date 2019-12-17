/**
 * @file link.h
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
#ifndef TESSERACT_URDF_LINK_H
#define TESSERACT_URDF_LINK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/inertial.h>
#include <tesseract_urdf/visual.h>
#include <tesseract_urdf/collision.h>

namespace tesseract_urdf
{
class LinkStatusCategory : public tesseract_common::StatusCategory
{
public:
  LinkStatusCategory(std::string link_name = "") : name_("LinkStatusCategory"), link_name_(std::move(link_name)) {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed link '" + link_name_ + "'!";
      case ErrorAttributeName:
        return "Missing or failed parsing link attribute 'name' for link '" + link_name_ + "'!";
      case ErrorParsingInertialElement:
        return "Error parsing link 'inertial' element for link '" + link_name_ + "'!";
      case ErrorParsingVisualElement:
        return "Error parsing link 'visual' element for link '" + link_name_ + "'!";
      case ErrorParsingCollisionElement:
        return "Error parsing link 'collision' element for link '" + link_name_ + "'!";
      default:
        return "Invalid error code for " + name_ + " for link '" + link_name_ + "'!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeName = -1,
    ErrorParsingInertialElement = -2,
    ErrorParsingVisualElement = -3,
    ErrorParsingCollisionElement = -4
  };

private:
  std::string name_;
  std::string link_name_;
};

inline tesseract_common::StatusCode::Ptr
parse(tesseract_scene_graph::Link::Ptr& link,
      const tinyxml2::XMLElement* xml_element,
      const tesseract_scene_graph::ResourceLocator::Ptr& locator,
      std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
      const int version)
{
  link = nullptr;
  auto status_cat = std::make_shared<LinkStatusCategory>();

  std::string link_name;
  if (QueryStringAttribute(xml_element, "name", link_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(LinkStatusCategory::ErrorAttributeName, status_cat);

  status_cat = std::make_shared<LinkStatusCategory>(link_name);
  auto l = std::make_shared<tesseract_scene_graph::Link>(link_name);

  // get inertia if it exists
  const tinyxml2::XMLElement* inertial = xml_element->FirstChildElement("inertial");
  if (inertial != nullptr)
  {
    auto status = parse(l->inertial, inertial, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          LinkStatusCategory::ErrorParsingInertialElement, status_cat, status);
  }

  // get visual if it exists
  for (const tinyxml2::XMLElement* visual = xml_element->FirstChildElement("visual"); visual;
       visual = xml_element->NextSiblingElement("visual"))
  {
    std::vector<tesseract_scene_graph::Visual::Ptr> temp_visual;
    auto status = parse(temp_visual, visual, locator, available_materials, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          LinkStatusCategory::ErrorParsingVisualElement, status_cat, status);

    l->visual.insert(l->visual.end(), temp_visual.begin(), temp_visual.end());
  }

  // get collision if exists
  for (const tinyxml2::XMLElement* collision = xml_element->FirstChildElement("collision"); collision;
       collision = collision->NextSiblingElement("collision"))
  {
    std::vector<tesseract_scene_graph::Collision::Ptr> temp_collision;
    auto status = parse(temp_collision, collision, locator, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          LinkStatusCategory::ErrorParsingCollisionElement, status_cat, status);

    l->collision.insert(l->collision.end(), temp_collision.begin(), temp_collision.end());
  }

  link = std::move(l);

  return std::make_shared<tesseract_common::StatusCode>(LinkStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf
#endif  // TESSERACT_URDF_LINK_H
