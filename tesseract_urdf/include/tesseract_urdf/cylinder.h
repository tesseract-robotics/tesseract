/**
 * @file cylinder.h
 * @brief Parse cylinder from xml string
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
#ifndef TESSERACT_URDF_CYLINDER_H
#define TESSERACT_URDF_CYLINDER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2
namespace tesseract_geometry
{
class Cylinder;
}

namespace tesseract_urdf
{
/**
 * @brief Parse a xml cylinder element
 * @param xml_element The xml element
 * @return Tesseract Geometry Cylinder
 */
std::shared_ptr<tesseract_geometry::Cylinder> parseCylinder(const tinyxml2::XMLElement* xml_element, int version);

tinyxml2::XMLElement* writeCylinder(const std::shared_ptr<const tesseract_geometry::Cylinder>& cylinder,
                                    tinyxml2::XMLDocument& doc);

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_CYLINDER_H
