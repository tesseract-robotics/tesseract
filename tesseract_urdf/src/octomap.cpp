/**
 * @file octomap.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>

#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_urdf/octomap.h>
#include <tesseract_urdf/octree.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/utils.h>

#ifdef TESSERACT_PARSE_POINT_CLOUDS
#include <tesseract_urdf/point_cloud.h>
#endif

tesseract_geometry::Octree::Ptr tesseract_urdf::parseOctomap(const tinyxml2::XMLElement* xml_element,
                                                             const tesseract_common::ResourceLocator& locator,
                                                             const bool /*visual*/,
                                                             int version)
{
  std::string shape_type;
  if (tesseract_common::QueryStringAttribute(xml_element, "shape_type", shape_type) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Octomap: Missing or failed parsing attribute 'shape_type'!"));

  tesseract_geometry::Octree::SubType sub_type{ tesseract_geometry::Octree::SubType::BOX };
  if (shape_type == "box")
    sub_type = tesseract_geometry::Octree::SubType::BOX;
  else if (shape_type == "sphere_inside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_INSIDE;
  else if (shape_type == "sphere_outside")
    sub_type = tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE;
  else
    std::throw_with_nested(std::runtime_error("Octomap: Invalid sub shape type, must be 'box', 'sphere_inside', or "
                                              "'sphere_outside'!"));

  bool prune = false;
  xml_element->QueryBoolAttribute("prune", &prune);

  const tinyxml2::XMLElement* octree_element = xml_element->FirstChildElement("octree");
  if (octree_element != nullptr)
  {
    try
    {
      return parseOctree(octree_element, locator, sub_type, prune, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Octomap: Failed parsing element 'octree'"));
    }
  }

#ifdef TESSERACT_PARSE_POINT_CLOUDS
  const tinyxml2::XMLElement* pcd_element = xml_element->FirstChildElement("point_cloud");
  if (pcd_element != nullptr)
  {
    try
    {
      return parsePointCloud(pcd_element, locator, sub_type, prune, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Octomap: Failed parsing element 'pointcloud'"));
    }
  }
#endif

  std::throw_with_nested(std::runtime_error("Octomap: Missing element 'octree' or 'point_cloud', must define one!"));
}

tinyxml2::XMLElement* tesseract_urdf::writeOctomap(const std::shared_ptr<const tesseract_geometry::Octree>& octree,
                                                   tinyxml2::XMLDocument& doc,
                                                   const std::string& package_path,
                                                   const std::string& filename)
{
  if (octree == nullptr)
    std::throw_with_nested(std::runtime_error("Octree is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("octree");

  std::string type_string;
  if (octree->getSubType() == tesseract_geometry::Octree::SubType::BOX)
    type_string = "box";
  else if (octree->getSubType() == tesseract_geometry::Octree::SubType::SPHERE_INSIDE)
    type_string = "sphere_inside";
  else if (octree->getSubType() == tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE)
    type_string = "sphere_outside";
  else
    std::throw_with_nested(std::runtime_error("Octree subtype is invalid and cannot be converted to XML"));
  xml_element->SetAttribute("shape_type", type_string.c_str());

  xml_element->SetAttribute("prune", octree->getPruned());

  try
  {
    tinyxml2::XMLElement* xml_octree = writeOctree(octree, doc, package_path, filename);
    xml_element->InsertEndChild(xml_octree);
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Octomap: Could not write octree to file"));
  }

  return xml_element;
}
