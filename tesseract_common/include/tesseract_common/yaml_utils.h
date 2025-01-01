/**
 * @file yaml_utils.h
 * @brief YAML Type conversions
 *
 * @author Levi Armstrong
 * @date September 5, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_YAML_UTILS_H
#define TESSERACT_COMMON_YAML_UTILS_H

#include <string>
#include <set>

#include <tesseract_common/filesystem.h>

namespace YAML
{
class Node;
}

namespace tesseract_common
{
class ResourceLocator;

/**
 * @brief Loads a YAML file and processes `!include` directives recursively.
 *
 * This function loads a YAML file and replaces any node tagged with `!include`
 * with the content of the specified file. It handles nested `!include` directives
 * and works with both maps and sequences.
 *
 * @param file_path The path to the YAML file to be loaded. Relative paths should be
 *                 resolved based on the caller's context.
 * @param locator The locator used to resolve urls and relative file paths.
 * @return A YAML::Node object containing the fully processed YAML structure.
 *
 * @throws std::runtime_error if an `!include` tag is used improperly (e.g., not scalar),
 *         or if a file specified in an `!include` directive cannot be loaded.
 */
YAML::Node loadYamlFile(const std::string& file_path, const ResourceLocator& locator);
YAML::Node loadYamlString(const std::string& yaml_string, const ResourceLocator& locator);

/**
 * @brief Writes a YAML::Node to a file.
 *
 * @param node The YAML::Node to write.
 * @param file_path The path to the output file.
 *
 * @throws std::runtime_error if the file cannot be opened.
 */
void writeYamlToFile(const YAML::Node& node, const std::string& file_path);

/**
 * @brief Check node map for unknown keys
 * @param node The node to check
 * @param expected_keys The expected keys
 */
void checkForUnknownKeys(const YAML::Node& node, const std::set<std::string>& expected_keys);

/**
 * @brief Converts a YAML::Node to a yaml string
 * @param node Input node
 * @return String containing the yaml
 */
std::string toYAMLString(const YAML::Node& node);

/**
 * @brief Converts yaml string to a YAML::Node
 * @param string Input string containing the yaml
 * @return Resulting YAML::Node
 */
YAML::Node fromYAMLString(const std::string& string);

/**
 * @brief Checks if the YAML::Nodes are identical
 * @details The == operator checks if they reference the same memory. This checks if they contain the same information
 * @param node1 Input YAML::Node
 * @param node2 Input YAML::Node
 */
bool compareYAML(const YAML::Node& node1, const YAML::Node& node2);
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_YAML_UTILS_H
