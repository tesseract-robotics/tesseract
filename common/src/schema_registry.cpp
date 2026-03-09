/**
 * @file schema_registry.cpp
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

#include <tesseract/common/schema_registry.h>
#include <tesseract/common/property_tree.h>

#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace tesseract::common
{
std::shared_ptr<SchemaRegistry> SchemaRegistry::instance()
{
  // local statics, not global variables
  static std::once_flag flag;
  static std::shared_ptr<SchemaRegistry> singleton;

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  std::call_once(flag, []() { singleton.reset(new SchemaRegistry()); });
  return singleton;
}

void SchemaRegistry::registerSchema(const std::string& key, const PropertyTree& schema)
{
  std::lock_guard lock(mutex_);
  schemas_[key] = schema;
}

void SchemaRegistry::registerSchemaFromFile(const std::string& key, const std::string& path)
{
  std::lock_guard lock(mutex_);
  paths_[key] = path;
  // Parse immediately and store
  YAML::Node node = YAML::LoadFile(path);
  schemas_[key] = PropertyTree::fromYAML(node);
}

bool SchemaRegistry::contains(const std::string& key) const
{
  std::lock_guard lock(mutex_);
  return schemas_.count(key) > 0 || paths_.count(key) > 0;
}

PropertyTree SchemaRegistry::get(const std::string& key) const
{
  std::lock_guard lock(mutex_);
  auto sit = schemas_.find(key);
  if (sit != schemas_.end())
    return sit->second;
  auto pit = paths_.find(key);
  if (pit != paths_.end())
  {
    // Lazy-load if path only was registered
    YAML::Node node = YAML::LoadFile(pit->second);
    schemas_[key] = PropertyTree::fromYAML(node);
    return schemas_.at(key);
  }
  throw std::out_of_range("SchemaRegistry: key not found -> " + key);
}

PropertyTree SchemaRegistry::loadFile(const std::string& path)
{
  // Allow relative or absolute paths
  std::filesystem::path p(path);
  if (!p.is_absolute())
  {
    // Optionally resolve against current working directory
    p = std::filesystem::current_path() / p;
  }
  YAML::Node node = YAML::LoadFile(p.string());
  return PropertyTree::fromYAML(node);
}

void SchemaRegistry::registerDerivedType(const std::string& base_type_name, const std::string& derived_type_name)
{
  std::lock_guard lock(mutex_);
  derived_types_[base_type_name].push_back(derived_type_name);
}

bool SchemaRegistry::isDerivedFrom(const std::string& base_type_name, const std::string& candidate_type) const
{
  std::lock_guard lock(mutex_);

  // A type is always compatible with itself
  if (base_type_name == candidate_type)
    return true;

  // Check if candidate is in the list of derived types for base
  auto it = derived_types_.find(base_type_name);
  if (it != derived_types_.end())
  {
    for (const auto& derived : it->second)
    {
      if (derived == candidate_type)
        return true;
    }
  }

  return false;
}

std::vector<std::string> SchemaRegistry::getDerivedTypes(const std::string& base_type_name) const
{
  std::lock_guard lock(mutex_);

  std::vector<std::string> result;
  result.push_back(base_type_name);  // Include the base type itself

  auto it = derived_types_.find(base_type_name);
  if (it != derived_types_.end())
  {
    for (const auto& derived : it->second)
      result.push_back(derived);
  }

  return result;
}

}  // namespace tesseract::common
