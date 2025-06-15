#ifndef TESSERACT_COMMON_PROPERTY_TREE_H
#define TESSERACT_COMMON_PROPERTY_TREE_H

#include <string>
#include <map>
#include <optional>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace tesseract_common
{
namespace property_type
{
constexpr std::string_view BOOL{ "bool" };
constexpr std::string_view STRING{ "string" };
constexpr std::string_view INT{ "int" };
constexpr std::string_view FLAOT{ "float" };
}  // namespace property_type

namespace property_attribute
{
constexpr std::string_view REQUIRED{ "required" };
constexpr std::string_view DEFAULT{ "default" };
constexpr std::string_view ENUM{ "enum" };
constexpr std::string_view MINIMUM{ "minimum" };
constexpr std::string_view MAXIMUM{ "maximum" };
}  // namespace property_attribute

/**
 * @brief A hierarchical property tree that stores values, metadata attributes,
 *        and supports schema merging, default handling, and custom validation.
 *
 * Each node may contain:
 *  - A YAML::Node value (scalar, sequence, or map)
 *  - Metadata attributes (as a map of YAML::Node)
 *  - Child PropertyTree nodes for nested structures
 *  - Custom validator functions to enforce constraints
 */
class PropertyTree
{
public:
  /**
   * @brief Signature for custom validator functions.
   * @param node   The PropertyTree node being validated.
   */
  using ValidatorFn = std::function<void(const PropertyTree&)>;

  /**
   * @brief Default constructor.
   */
  PropertyTree() = default;

  /**
   * @brief Merge a schema tree into this node.
   *    * Copies schema attributes, applies non-required defaults,
   * merges child structure, and registers schema validators.
   * @param schema The schema node to merge from.
   */
  void mergeSchema(const PropertyTree& schema);

  /**
   * @brief Validate this tree using registered validators.
   *    * Traverses the tree, invoking each node's validators.
   */
  void validate() const;

  /**
   * @brief Register a custom validator for this node.
   * @param fn Validator function to invoke during validate().
   */
  void addValidator(ValidatorFn fn);

  /**
   * @brief Access or create a child node by key.
   * @param key Child node identifier.
   * @return Reference to the child node.
   */
  PropertyTree& get(std::string_view key);

  /**
   * @brief Access a child node by key (const).
   * @param key Child node identifier.
   * @return Const reference to the child node.
   */
  const PropertyTree& get(std::string_view key) const;

  /**
   * @brief Find a child node without creating it.
   * @param key Child node identifier.
   * @return Pointer to the child or nullptr if missing.
   */
  const PropertyTree* find(std::string_view key) const;

  /**
   * @brief Set the YAML value for this node.
   * @param v YAML::Node representing the value.
   */
  void setValue(const YAML::Node& v);

  /**
   * @brief Get the YAML value stored in this node.
   * @return Const reference to YAML::Node.
   */
  const YAML::Node& getValue() const;

  /**
   * @brief List all immediate child keys.
   * @return Vector of child key strings.
   */
  std::vector<std::string> keys() const;

  /**
   * @brief Set a metadata attribute (YAML::Node form).
   * @param name  Attribute name.
   * @param attr  YAML::Node value.
   */
  void setAttribute(std::string_view name, const YAML::Node& attr);

  /**
   * @brief Set a string attribute.
   * @param name  Attribute name.
   * @param attr  String value.
   */
  void setAttribute(std::string_view name, std::string_view attr);

  /**
   * @brief Set a C-string attribute.
   */
  void setAttribute(std::string_view name, const char* attr);

  /**
   * @brief Set a boolean attribute.
   */
  void setAttribute(std::string_view name, bool attr);

  /**
   * @brief Set an integer attribute.
   */
  void setAttribute(std::string_view name, int attr);

  /**
   * @brief Set a double attribute.
   */
  void setAttribute(std::string_view name, double attr);

  /**
   * @brief Check if an attribute exists and is non-null.
   * @param name Attribute name.
   * @return True if present and non-null.
   */
  bool hasAttribute(std::string_view name) const;

  /**
   * @brief Retrieve an attribute value.
   * @param name Attribute name.
   * @return Optional<YAML::Node> if attribute exists.
   */
  std::optional<YAML::Node> getAttribute(std::string_view name) const;

  /**
   * @brief List all metadata attribute keys.
   * @return Vector of attribute key strings.
   */
  std::vector<std::string> getAttributeKeys() const;

  /**
   * @brief Build a PropertyTree from a YAML::Node.
   * @param node Root YAML::Node.
   * @return Populated PropertyTree hierarchy.
   */
  static PropertyTree fromYAML(const YAML::Node& node);

  /**
   * @brief Serialize this tree to a YAML::Node.
   * @param exclude_attributes If true, omit the attributes map.
   * @return YAML::Node representation of the tree.
   */
  YAML::Node toYAML(bool exclude_attributes = true) const;

private:
  YAML::Node value_;                             /**< Stored YAML value */
  std::map<std::string, YAML::Node> attributes_; /**< Metadata attributes */
  std::map<std::string, PropertyTree> children_; /**< Nested child nodes */
  std::vector<ValidatorFn> validators_;          /**< Registered validators */
};

/**
 * @brief Ensure a required attribute exists and is non-null.
 * @param node PropertyTree node to validate.
 */
void validateRequired(const PropertyTree& node);

/**
 * @brief Enforce numeric range if 'minimum'/'maximum' attributes are set.
 * @param node PropertyTree node to validate.
 */
void validateRange(const PropertyTree& node);

/**
 * @brief Enforce that the node's value is one of the 'enum' attribute list.
 * @param node PropertyTree node to validate.
 */
void validateEnum(const PropertyTree& node);

}  // namespace tesseract_common

#endif  // PROPERTY_TREE_H
