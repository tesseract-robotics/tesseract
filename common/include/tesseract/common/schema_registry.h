/**
 * @file schema_registry.h
 * @brief This is a schema registry class
 *
 * @author Levi Armstrong
 * @date March 1, 2026
 *
 * @copyright Copyright (c) 2026, Levi Armstrong
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
#ifndef TESSERACT_COMMON_SCHEMA_REGISTRY_H
#define TESSERACT_COMMON_SCHEMA_REGISTRY_H

#include <string>
#include <map>
#include <mutex>
#include <memory>

namespace tesseract::common
{
class PropertyTree;

/**
 * @brief A global registry of named schemas for PropertyTree.
 *
 * Use SchemaRegistry::instance() to access the singleton.
 */
class SchemaRegistry
{
public:
  ~SchemaRegistry() = default;
  SchemaRegistry(const SchemaRegistry&) = delete;
  SchemaRegistry& operator=(const SchemaRegistry&) = delete;
  SchemaRegistry(const SchemaRegistry&&) = delete;
  SchemaRegistry& operator=(const SchemaRegistry&&) = delete;

  /** @brief Access the singleton instance. */
  static std::shared_ptr<SchemaRegistry> instance();

  /**
   * @brief Register an already-parsed schema under a logical key.
   * @param key    Unique identifier for this schema.
   * @param schema Parsed PropertyTree schema.
   */
  void registerSchema(const std::string& key, const PropertyTree& schema);

  /**
   * @brief Register a schema from a YAML file path.
   * @param key   Unique identifier for this schema.
   * @param path  Path to a .yaml schema file.
   */
  void registerSchemaFromFile(const std::string& key, const std::string& path);

  /**
   * @brief Check whether a schema is registered under this key.
   * @param key  Logical identifier.
   * @return True if present.
   */
  bool contains(const std::string& key) const;

  /**
   * @brief Retrieve a registered schema by key.
   * @param key Logical identifier.
   * @return Copy of the schema PropertyTree.
   * @throws std::out_of_range if the key is not found.
   */
  PropertyTree get(const std::string& key) const;

  /**
   * @brief Load and parse an arbitrary YAML file into a PropertyTree.
   * @param path  Path to a .yaml file.
   * @return Parsed PropertyTree.
   * @throws YAML::Exception on parse errors.
   */
  static PropertyTree loadFile(const std::string& path);

private:
  SchemaRegistry() = default;

  mutable std::mutex mutex_;
  mutable std::map<std::string, PropertyTree> schemas_;
  mutable std::map<std::string, std::string> paths_;
};

}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_SCHEMA_REGISTRY_H
