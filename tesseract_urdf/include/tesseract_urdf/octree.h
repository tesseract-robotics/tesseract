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
#include <exception>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/array.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_urdf
{
inline tesseract_geometry::Octree::Ptr parseOctree(const tinyxml2::XMLElement* xml_element,
                                                   const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                                   tesseract_geometry::Octree::SubType shape_type,
                                                   const bool prune,
                                                   const int /*version*/)
{
  std::string filename;
  if (tesseract_common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Octree: Missing or failed parsing attribute 'filename'!"));

  tesseract_common::Resource::Ptr resource = locator->locateResource(filename);
  if (!resource || !resource->isFile())
    std::throw_with_nested(std::runtime_error("Octree: Missing resource '" + filename + "'!"));

  auto ot = std::make_shared<octomap::OcTree>(resource->getFilePath());

  if (ot == nullptr || ot->size() == 0)
    std::throw_with_nested(std::runtime_error("Octree: Error importing from '" + filename + "'!"));

  if (prune)
    tesseract_geometry::Octree::prune(*ot);

  auto geom = std::make_shared<tesseract_geometry::Octree>(ot, shape_type);
  if (geom == nullptr)
    std::throw_with_nested(std::runtime_error("Octree: Error creating octree geometry type from octomap::octree!"));

  return geom;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_OCTREE_H
