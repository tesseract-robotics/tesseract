/**
 * @file box.h
 * @brief Parse box from xml string
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
#ifndef TESSERACT_URDF_SDF_MESH_H
#define TESSERACT_URDF_SDF_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract_common
{
class ResourceLocator;
}

namespace tesseract_geometry
{
class SDFMesh;
}

namespace tesseract_urdf
{
/**
 * @brief Parse xml element sdf_mesh
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @param visual Indicate if visual
 * @param version The version number
 * @return A vector of Tesseract SDFMeshes
 */
std::vector<std::shared_ptr<tesseract_geometry::SDFMesh>> parseSDFMesh(const tinyxml2::XMLElement* xml_element,
                                                                       const tesseract_common::ResourceLocator& locator,
                                                                       bool visual,
                                                                       int version);

/**
 * @brief writeSDFMesh Write SDF Mesh to URDF XML.  This is non-standard URDF / tesseract-exclusive
 * @param sdf_mesh Mesh to be written
 * @param doc XML Document to manage generated XML
 * @param package_path /<path>/<to>/<your-package>.  If set, geometry will be saved relative to the package.  If not
 * set, geometry will be saved with absolute paths.
 * @param filename Desired file location.  If package_path is set, this should be relative to the package.  Otherwise,
 * this should be an absolute path
 * @return XML element representing the sdf mesh object in URDF format
 */
tinyxml2::XMLElement* writeSDFMesh(const std::shared_ptr<const tesseract_geometry::SDFMesh>& sdf_mesh,
                                   tinyxml2::XMLDocument& doc,
                                   const std::string& package_path,
                                   const std::string& filename);

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_SDF_MESH_H
