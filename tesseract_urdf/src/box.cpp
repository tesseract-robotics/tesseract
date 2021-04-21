/**
 * @file box.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <tesseract_common/utils.h>
#include <boost/algorithm/string.hpp>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/box.h>
#include <tesseract_geometry/impl/box.h>

tesseract_geometry::Box::Ptr tesseract_urdf::parseBox(const tinyxml2::XMLElement* xml_element, int /*version*/)
{
  std::string size_string;
  if (tesseract_common::QueryStringAttribute(xml_element, "size", size_string) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Box: Missing or failed parsing box attribute size!"));

  std::vector<std::string> tokens;
  boost::split(tokens, size_string, boost::is_any_of(" "), boost::token_compress_on);
  if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
    std::throw_with_nested(std::runtime_error("Box: Failed converting box attribute size to vector!"));

  double l{ 0 }, w{ 0 }, h{ 0 };
  // No need to check return values because the tokens are verified above
  tesseract_common::toNumeric<double>(tokens[0], l);
  tesseract_common::toNumeric<double>(tokens[1], w);
  tesseract_common::toNumeric<double>(tokens[2], h);

  if (!(l > 0))
    std::throw_with_nested(std::runtime_error("Box: The length must be greater than zero!"));

  if (!(w > 0))
    std::throw_with_nested(std::runtime_error("Box: The width must be greater than zero!"));

  if (!(h > 0))
    std::throw_with_nested(std::runtime_error("Box: The height must be greater than zero!"));

  return std::make_shared<tesseract_geometry::Box>(l, w, h);
}
