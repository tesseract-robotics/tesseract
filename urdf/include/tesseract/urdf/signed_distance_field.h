/**
 * @file signed_distance_field.h
 * @brief Parse signed distance field from xml string
 *
 * @author Joel Kang
 * @date June 26, 2026
 *
 * @copyright Copyright (c) 2026, Tesseract
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
#ifndef TESSERACT_URDF_SIGNED_DISTANCE_FIELD_H
#define TESSERACT_URDF_SIGNED_DISTANCE_FIELD_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string_view>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <tesseract/geometry/fwd.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract::urdf
{
static constexpr std::string_view SIGNED_DISTANCE_FIELD_ELEMENT_NAME = "tesseract:signed_distance_field";

/**
 * @brief Parse xml element signed_distance_field
 *
 * Loads a serialized OpenVDB FloatGrid from the resource referenced by the @c filename attribute.
 * This is the VDB binary distance-field format, not Gazebo's "SDF" Simulation Description Format.
 *
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @return A Tesseract SignedDistanceField geometry
 */
std::shared_ptr<tesseract::geometry::SignedDistanceField>
parseSignedDistanceField(const tinyxml2::XMLElement* xml_element, const tesseract::common::ResourceLocator& locator);

/**
 * @brief writeSignedDistanceField Write a signed distance field out to file, and generate appropriate xml.  This is
 * non-standard URDF / tesseract-exclusive
 * @param sdf The geometry element containing the signed distance field
 * @param doc The XML document to which to add the xml data
 * @param package_path /<path>/<to>/<your-package>.  If set, geometry will be saved relative to the package.  If not
 * set, geometry will be saved with absolute paths.
 * @param filename Desired filename relative to the working directory ("sdf.vdb" or "collision/sdf.vdb")
 * @return An XML element containing information on the saved file.
 */
tinyxml2::XMLElement*
writeSignedDistanceField(const std::shared_ptr<const tesseract::geometry::SignedDistanceField>& sdf,
                         tinyxml2::XMLDocument& doc,
                         const std::string& package_path,
                         const std::string& filename);

}  // namespace tesseract::urdf

#endif  // TESSERACT_URDF_SIGNED_DISTANCE_FIELD_H
