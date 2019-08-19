/**
 * @file octree.h
 * @brief Parse octree from xml string
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_SCENE_GRAPH_URDF_PARSER_OCTREE_H
#define TESSERACT_SCENE_GRAPH_URDF_PARSER_OCTREE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser/utils.h>

namespace tesseract_scene_graph
{

class OctreeStatusCategory : public tesseract_common::StatusCategory
{
public:
  OctreeStatusCategory() : name_("OctreeStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed octree element!";
      case ErrorAttributeFileName:
        return "Missing or failed parsing octree attribute 'filename'!";
      case ErrorAttributeShapeType:
        return "Missing or failed parsing octree attribute 'shape_type'!";
      case ErrorInvalidShapeType:
        return "Invalide sub shape type for octree attribute 'shape_type', must be 'box', 'sphere_inside', or 'sphere_outside'!";
      case ErrorParsingAttributeScale:
        return "Failed parsing octree attribute 'scale'!";
      case ErrorImportingOctree:
        return "Error importing octree from filename!";
      case ErrorCreatingGeometry:
        return "Error create octree geometry type from octomap::octree!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeFileName = -1,
    ErrorAttributeShapeType = -2,
    ErrorInvalidShapeType = -3,
    ErrorParsingAttributeScale = -4,
    ErrorImportingOctree = -5,
    ErrorCreatingGeometry = -6
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_geometry::Octree::Ptr& octree,
                                               const tinyxml2::XMLElement* xml_element,
                                               ResourceLocatorFn locator,
                                               bool /*visual*/ = true)
{
  octree = nullptr;
  auto status_cat = std::make_shared<OctreeStatusCategory>();

  std::string filename;
  if (QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorAttributeFileName, status_cat);

  std::string shape_type;
  if (QueryStringAttribute(xml_element, "shape_type", shape_type) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorAttributeShapeType, status_cat);

  tesseract_geometry::Octree::SubType sub_type;
  if (shape_type == "box")
    sub_type = tesseract_geometry::Octree::SubType::BOX;
  else if (shape_type == "sphere_inside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_INSIDE;
  else if (shape_type == "sphere_outside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE;
  else
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorInvalidShapeType, status_cat);

  std::string scale_string;
  Eigen::Vector3d scale(1, 1, 1);
  if (QueryStringAttribute(xml_element, "scale", scale_string) == tinyxml2::XML_SUCCESS)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, scale_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
      return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorParsingAttributeScale, status_cat);

    scale = Eigen::Vector3d(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
  }

  auto ot = std::make_shared<octomap::OcTree>(locator(filename));
  if (ot == nullptr || ot->size() == 0)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorImportingOctree, status_cat);

//  if (visual)
//    // Should not prune
//  else
//    // Should prune

  auto geom = std::make_shared<tesseract_geometry::Octree>(ot, sub_type);
  if (geom == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorCreatingGeometry, status_cat);


  octree = std::move(geom);
  return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::Success, status_cat);;
}

}

#endif // TESSERACT_SCENE_GRAPH_URDF_PARSER_OCTREE_H
