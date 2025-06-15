#ifndef TESSERACT_COMMON_SCHEMA_REGISTRY_H
#define TESSERACT_COMMON_SCHEMA_REGISTRY_H

#include <string>
#include <map>
#include <mutex>
#include <memory>

namespace tesseract_common
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
   * @return Const reference to the schema PropertyTree.
   * @throws std::out_of_range if the key is not found.
   */
  const PropertyTree& get(const std::string& key) const;

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

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_SCHEMA_REGISTRY_H
