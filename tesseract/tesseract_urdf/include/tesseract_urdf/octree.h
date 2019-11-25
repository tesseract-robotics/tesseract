/**
 * @file octree.h
 * @brief Parse octree from xml string
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
#ifndef TESSERACT_URDF_OCTREE_H
#define TESSERACT_URDF_OCTREE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/array.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
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
      case ErrorImportingOctree:
        return "Error importing octree from 'filename'!";
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
    ErrorImportingOctree = -2,
    ErrorCreatingGeometry = -3
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parseOctree(tesseract_geometry::Octree::Ptr& octree,
                                                     const tinyxml2::XMLElement* xml_element,
                                                     const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                                     tesseract_geometry::Octree::SubType shape_type,
                                                     const bool prune,
                                                     const int /*version*/)
{
  octree = nullptr;
  auto status_cat = std::make_shared<OctreeStatusCategory>();

  std::string filename;
  if (QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorAttributeFileName, status_cat);

  tesseract_common::Resource::Ptr resource = locator->locateResource(filename);
  if (!resource)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorImportingOctree, status_cat);
  if (!resource->isFile())
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorImportingOctree, status_cat);

  auto ot = std::make_shared<octomap::OcTree>(resource->getFilePath());

  if (ot == nullptr || ot->size() == 0)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorImportingOctree, status_cat);

  if (prune)
    tesseract_geometry::Octree::prune(*ot);

  auto geom = std::make_shared<tesseract_geometry::Octree>(ot, shape_type);
  if (geom == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::ErrorCreatingGeometry, status_cat);

  octree = std::move(geom);
  return std::make_shared<tesseract_common::StatusCode>(OctreeStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_OCTREE_H
