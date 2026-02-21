/**
 * @file octree.h
 * @brief Parse octree from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
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
#include <memory>
#include <string_view>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_geometry/fwd.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract::urdf
{
static constexpr std::string_view OCTREE_ELEMENT_NAME = "tesseract:octree";

/**
 * @brief Parse xml element octree
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @param shape_type The collision/visual shape type to use
 * @param prune Indicate if the octree should be pruned
 * @return A Tesseract Geometry Octree
 */
std::shared_ptr<tesseract::geometry::Octree> parseOctree(const tinyxml2::XMLElement* xml_element,
                                                         const tesseract::common::ResourceLocator& locator,
                                                         tesseract::geometry::OctreeSubType shape_type,
                                                         bool prune);

/**
 * @brief writeOctree Write octree out to file, and generate appropriate xml
 * @param octree The geometry element containing octree data
 * @param doc The XML document to which to add the xml data
 * @param package_path /<path>/<to>/<your-package>.  If set, geometry will be saved relative to the package.  If not
 * set, geometry will be saved with absolute paths.
 * @param filename Desired filename relative to the working directory ("octree.ot" or "collision/octree.ot")
 * @return An XML element containing information on the saved file.
 */
tinyxml2::XMLElement* writeOctree(const std::shared_ptr<const tesseract::geometry::Octree>& octree,
                                  tinyxml2::XMLDocument& doc,
                                  const std::string& package_path,
                                  const std::string& filename);

}  // namespace tesseract::urdf

#endif  // TESSERACT_URDF_OCTREE_H
