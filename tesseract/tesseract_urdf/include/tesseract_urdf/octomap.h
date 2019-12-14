/**
 * @file octomap.h
 * @brief Parse octomap from xml string
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
#ifndef TESSERACT_URDF_OCTOMAP_H
#define TESSERACT_URDF_OCTOMAP_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/octree.h>

#ifdef TESSERACT_PARSE_POINT_CLOUDS
#include <tesseract_urdf/point_cloud.h>
#endif

namespace tesseract_urdf
{
class OctomapStatusCategory : public tesseract_common::StatusCategory
{
public:
  OctomapStatusCategory() : name_("OctomapStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed octree element!";
      case ErrorAttributeShapeType:
        return "Missing or failed parsing octomap attribute 'shape_type'!";
      case ErrorInvalidShapeType:
        return "Invalide sub shape type for octoamp attribute 'shape_type', must be 'box', 'sphere_inside', or "
               "'sphere_outside'!";
      case ErrorMissingOctreeOrPointCloudElement:
        return "Missing octomap element 'octree' or 'point_cloud', must define one!";
      case ErrorParsingOctreeElement:
        return "Failed parsing octomap element 'octree'!";
      case ErrorParsingPointCloudElement:
        return "Failed parsing octomap element 'pointcloud'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeShapeType = -1,
    ErrorInvalidShapeType = -2,
    ErrorMissingOctreeOrPointCloudElement = -3,
    ErrorParsingOctreeElement = -4,
    ErrorParsingPointCloudElement = -5
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_geometry::Octree::Ptr& octree,
                                               const tinyxml2::XMLElement* xml_element,
                                               const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                               const bool /*visual*/,
                                               const int version)
{
  octree = nullptr;
  auto status_cat = std::make_shared<OctomapStatusCategory>();

  std::string shape_type;
  if (QueryStringAttribute(xml_element, "shape_type", shape_type) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(OctomapStatusCategory::ErrorAttributeShapeType, status_cat);

  tesseract_geometry::Octree::SubType sub_type;
  if (shape_type == "box")
    sub_type = tesseract_geometry::Octree::SubType::BOX;
  else if (shape_type == "sphere_inside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_INSIDE;
  else if (shape_type == "sphere_outside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE;
  else
    return std::make_shared<tesseract_common::StatusCode>(OctomapStatusCategory::ErrorInvalidShapeType, status_cat);

  bool prune = false;
  xml_element->QueryBoolAttribute("prune", &prune);

  const tinyxml2::XMLElement* octree_element = xml_element->FirstChildElement("octree");
  if (octree_element != nullptr)
  {
    auto status = parseOctree(octree, octree_element, locator, sub_type, prune, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          OctomapStatusCategory::ErrorParsingOctreeElement, status_cat, status);

    return std::make_shared<tesseract_common::StatusCode>(OctomapStatusCategory::Success, status_cat);
  }

#ifdef TESSERACT_PARSE_POINT_CLOUDS
  const tinyxml2::XMLElement* pcd_element = xml_element->FirstChildElement("point_cloud");
  if (pcd_element != nullptr)
  {
    auto status = parsePointCloud(octree, pcd_element, locator, sub_type, prune, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          OctomapStatusCategory::ErrorParsingPointCloudElement, status_cat, status);

    return std::make_shared<tesseract_common::StatusCode>(OctomapStatusCategory::Success, status_cat);
  }
#endif
  return std::make_shared<tesseract_common::StatusCode>(OctomapStatusCategory::ErrorMissingOctreeOrPointCloudElement,
                                                        status_cat);
}

}  // namespace tesseract_urdf
#endif  // TESSERACT_URDF_OCTOMAP_H
