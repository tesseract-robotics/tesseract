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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_common
{
/**
 * @brief Recursively processes a YAML node to resolve `!include` directives.
 *
 * This function replaces any node tagged with `!include` with the content of the
 * specified file. It also recursively processes maps and sequences to handle nested
 * `!include` directives.
 *
 * @param node The YAML node to process.
 * @param locator The locator used to resolve urls and relative file paths.
 * @return A YAML::Node with all `!include` directives resolved.
 *
 * @throws std::runtime_error if an `!include` tag is used improperly (e.g., not scalar),
 *         or if a file specified in an `!include` directive cannot be loaded.
 */
YAML::Node handleIncludeDirective(const YAML::Node& node, const ResourceLocator& locator)
{
  if (node.Tag() == "!include")
  {
    // Ensure the node is scalar and contains a file path
    if (!node.IsScalar())
      throw std::runtime_error("!include tag must be a scalar containing the file path");

    // Resolve the file path and load the included file
    auto included_file = node.as<std::string>();
    auto resource = locator.locateResource(included_file);
    if (resource == nullptr)
      throw std::runtime_error("Unable to locate resource: " + included_file);

    return handleIncludeDirective(YAML::LoadFile(resource->getFilePath()), *resource);
  }

  if (node.IsMap())
  {
    // Create a new map and process each key-value pair
    YAML::Node processed_map = YAML::Node(YAML::NodeType::Map);
    for (auto it = node.begin(); it != node.end(); ++it)
      processed_map[it->first] = handleIncludeDirective(it->second, locator);

    return processed_map;
  }

  if (node.IsSequence())
  {
    // Create a new sequence and process each element
    YAML::Node processed_sequence = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& child : node)
      processed_sequence.push_back(handleIncludeDirective(child, locator));

    return processed_sequence;
  }

  // Return the node as-is for scalars and unsupported types
  return node;
}

YAML::Node loadYamlFile(const std::string& file_path, const ResourceLocator& locator)
{
  // Load the base YAML file
  auto resource = locator.locateResource(file_path);
  YAML::Node root = YAML::LoadFile(resource->getFilePath());
  return handleIncludeDirective(root, *resource);
}

YAML::Node loadYamlString(const std::string& yaml_string, const ResourceLocator& locator)
{
  // Load the base YAML file
  YAML::Node root = YAML::Load(yaml_string);
  return handleIncludeDirective(root, locator);
}

void writeYamlToFile(const YAML::Node& node, const std::string& file_path)
{
  YAML::Emitter out;
  out << node;

  if (!out.good())
    throw std::runtime_error("Failed to serialize YAML node: " + std::string(out.GetLastError()));

  std::ofstream file(file_path);
  if (!file.is_open())
    throw std::runtime_error("Failed to open file: " + file_path);

  file << out.c_str();
  file.close();
}

void checkForUnknownKeys(const YAML::Node& node, const std::set<std::string>& expected_keys)
{
  if (!node.IsMap())
    throw std::runtime_error("checkForUnknownKeys, node should be a map");

  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    auto key = it->first.as<std::string>();
    if (expected_keys.find(key) == expected_keys.end())
      throw std::runtime_error("checkForUnknownKeys, unknown key: " + key);
  }
}

std::string toYAMLString(const YAML::Node& node)
{
  std::stringstream stream;
  stream << node;
  return stream.str();
}

YAML::Node fromYAMLString(const std::string& string) { return YAML::Load(string); }

bool compareYAML(const YAML::Node& node1, const YAML::Node& node2)
{
  if (node1.is(node2))
    return true;

  if (node1.Type() != node2.Type())
    return false;

  switch (node1.Type())
  {
    case YAML::NodeType::Scalar:
    {
      try
      {
        auto v1 = node1.as<bool>();
        auto v2 = node2.as<bool>();
        return (v1 == v2);
      }
      catch (YAML::TypedBadConversion<bool>& /*e*/)
      {
        try
        {
          auto v1 = node1.as<int>();
          auto v2 = node2.as<int>();
          return (v1 == v2);
        }
        catch (YAML::TypedBadConversion<int>& /*e*/)
        {
          try
          {
            auto v1 = node1.as<double>();
            auto v2 = node2.as<double>();
            return almostEqualRelativeAndAbs(v1, v2, 1e-6, std::numeric_limits<float>::epsilon());
          }
          catch (YAML::TypedBadConversion<double>& /*e*/)
          {
            try
            {
              auto v1 = node1.as<std::string>();
              auto v2 = node2.as<std::string>();
              return (v1 == v2);
            }
            catch (YAML::TypedBadConversion<std::string>& /*e*/)
            {
              return false;
            }
          }
        }
      }
    }
    case YAML::NodeType::Null:
      return true;
    case YAML::NodeType::Undefined:
      return false;
    case YAML::NodeType::Map:
    case YAML::NodeType::Sequence:
      if (node1.size() != node2.size())
        return false;
  }

  if (node1.IsMap())
  {
    bool result = true;
    for (YAML::const_iterator it1 = node1.begin(), it2; it1 != node1.end() && result; ++it1)
    {
      for (it2 = node2.begin(); it2 != node2.end(); ++it2)
      {
        if (compareYAML(it1->first, it2->first))
          break;
      }
      if (it2 == node2.end())
        return false;

      result = compareYAML(it1->second, it2->second);
    }
    return result;
  }

  return std::equal(node1.begin(), node1.end(), node2.begin(), compareYAML);
}
}  // namespace tesseract_common
