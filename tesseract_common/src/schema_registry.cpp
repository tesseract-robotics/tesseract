#include <tesseract_common/schema_registry.h>
#include <tesseract_common/property_tree.h>

#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace tesseract_common
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

const PropertyTree& SchemaRegistry::get(const std::string& key) const
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

}  // namespace tesseract_common
