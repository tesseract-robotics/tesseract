/**
 * @file urdf_parser.h
 * @brief A urdf parser for tesseract
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
#ifndef TESSERACT_URDF_URDF_PARSER_H
#define TESSERACT_URDF_URDF_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_urdf
{
/**
 * @brief Print a nested exception
 * @param e The exception to print
 * @param level The exception level which controls the indentation
 */
void printNestedException(const std::exception& e, int level = 0);

/**
 * @brief Parse a URDF string into a Tesseract Scene Graph
 * @param urdf_xml_string URDF xml string
 * @param locator The resource locator function
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string,
                                                       const tesseract_scene_graph::ResourceLocator::Ptr& locator);

/**
 * @brief Parse a URDF file into a Tesseract Scene Graph
 * @param URDF file path
 * @param The resource locator function
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::Ptr parseURDFFile(const std::string& path,
                                                     const tesseract_scene_graph::ResourceLocator::Ptr& locator);

}  // namespace tesseract_urdf

#endif
