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

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_urdf
{
/**
 * @brief Parse a URDF string into a Tesseract Scene Graph
 * @param urdf_xml_string URDF xml string
 * @param locator The resource locator function
 * @throws std::nested_exception Thrown if error occurs during parsing. Use printNestedException to print contents of
 * the nested exception.
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::UPtr parseURDFString(const std::string& urdf_xml_string,
                                                        const tesseract_common::ResourceLocator& locator);

/**
 * @brief Parse a URDF file into a Tesseract Scene Graph
 * @param URDF file path
 * @param The resource locator function
 * @throws std::nested_exception Thrown if error occurs during parsing. Use printNestedException to print contents of
 * the nested exception.
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::UPtr parseURDFFile(const std::string& path,
                                                      const tesseract_common::ResourceLocator& locator);

void writeURDFFile(const tesseract_scene_graph::SceneGraph::ConstPtr& sg,
                   const std::string& directory,
                   const std::string& filename);
}  // namespace tesseract_urdf

#endif
