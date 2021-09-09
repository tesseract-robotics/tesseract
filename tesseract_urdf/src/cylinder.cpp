/**
 * @file cylinder.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/cylinder.h>
#include <tesseract_geometry/impl/cylinder.h>

tesseract_geometry::Cylinder::Ptr tesseract_urdf::parseCylinder(const tinyxml2::XMLElement* xml_element,
                                                                int /*version*/)
{
  double r{ 0 }, l{ 0 };
  if (xml_element->QueryDoubleAttribute("length", &(l)) != tinyxml2::XML_SUCCESS || !(l > 0))
    std::throw_with_nested(std::runtime_error("Cylinder: Missing or failed parsing attribute 'length'!"));

  if (xml_element->QueryDoubleAttribute("radius", &(r)) != tinyxml2::XML_SUCCESS || !(r > 0))
    std::throw_with_nested(std::runtime_error("Cylinder: Missing or failed parsing attribute 'radius'!"));

  return std::make_shared<tesseract_geometry::Cylinder>(r, l);
}

tinyxml2::XMLElement* tesseract_urdf::writeCylinder(const std::shared_ptr<const tesseract_geometry::Cylinder>& cylinder,
                                                    tinyxml2::XMLDocument& doc)
{
  if (cylinder == nullptr)
    std::throw_with_nested(std::runtime_error("Cylinder is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("cylinder");
  xml_element->SetAttribute("length", cylinder->getLength());
  xml_element->SetAttribute("radius", cylinder->getRadius());
  return xml_element;
}
