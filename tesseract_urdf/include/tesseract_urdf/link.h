/**
 * @file link.h
 * @brief Parse link from xml string
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
#ifndef TESSERACT_URDF_LINK_H
#define TESSERACT_URDF_LINK_H

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

namespace tesseract_urdf
{
static constexpr std::string_view LINK_ELEMENT_NAME = "link";

/**
 * @brief Parse xml element link
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @param available_materials The current available materials
 * @param make_convex_meshes Flag to indicate if the meshes should be converted to convex hulls
 * @return A Tesseract Link
 */
std::shared_ptr<tesseract_scene_graph::Link>
parseLink(const tinyxml2::XMLElement* xml_element,
          const tesseract_common::ResourceLocator& locator,
          bool make_convex_meshes,
          std::unordered_map<std::string, std::shared_ptr<tesseract_scene_graph::Material>>& available_materials);

/**
 * @brief writeLink Write a link to URDF XML
 * @param link Link object to be written
 * @param doc XML Document to which element will belong
 * @param package_path /<path>/<to>/<your-package>.  If set, geometry will be saved relative to the package.  If not
 * set, geometry will be saved with absolute paths.
 * @return XML element representing link in URDF format
 */
tinyxml2::XMLElement* writeLink(const std::shared_ptr<const tesseract_scene_graph::Link>& link,
                                tinyxml2::XMLDocument& doc,
                                const std::string& package_path);

}  // namespace tesseract_urdf
#endif  // TESSERACT_URDF_LINK_H
