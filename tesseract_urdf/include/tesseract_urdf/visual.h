/**
 * @file visual.h
 * @brief Parse visual from xml string
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
#ifndef TESSERACT_URDF_VISUAL_H
#define TESSERACT_URDF_VISUAL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
#include <string_view>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_scene_graph/fwd.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract::urdf
{
static constexpr std::string_view VISUAL_ELEMENT_NAME = "visual";

/**
 * @brief Parse xml element visual
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @return A Visual object
 */
std::shared_ptr<tesseract::scene_graph::Visual>
parseVisual(const tinyxml2::XMLElement* xml_element,
            const tesseract::common::ResourceLocator& locator,
            std::unordered_map<std::string, std::shared_ptr<tesseract::scene_graph::Material>>& available_materials);

/**
 * @brief writeVisual Write one visual geometry object to URDF XML
 * @param visual Visual object to be written
 * @param doc XML Document to which XML will belong
 * @param package_path /<path>/<to>/<your-package>.  If set, geometry will be saved relative to the package.  If not
 * set, geometry will be saved with absolute paths.
 * @param link_name Name of link to which collision object is attached
 * @param id If set, this ID will be appended to the geometry name for distinguishing between multiple geometries on
 * the same link.
 * @return An XML element representing the collision object in URDF format.
 */
tinyxml2::XMLElement* writeVisual(const std::shared_ptr<const tesseract::scene_graph::Visual>& visual,
                                  tinyxml2::XMLDocument& doc,
                                  const std::string& package_path,
                                  const std::string& link_name,
                                  int id);

}  // namespace tesseract::urdf

#endif  // TESSERACT_URDF_VISUAL_H
