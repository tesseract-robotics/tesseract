#ifndef TESSERACT_COMMON_PROPERTY_TREE_H
#define TESSERACT_COMMON_PROPERTY_TREE_H

#include <string>
#include <string_view>
#include <map>
#include <vector>
#include <optional>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace tesseract_common
{
namespace property_type
{
/**
 * @brief A utility for constructing the list type
 * @param type The type assoicated with the list
 * @return The string representation of the std::vector<type>, aka. type[]
 */
std::string createList(std::string_view type);

/**
 * @brief A utility for constructing the map type
 * @param key_type The type assoicated with the map key
 * @param value_type The type assoicated with the map value
 * @return The string representation of the std::make<key_type, value_type>, aka. {key_type : value_type}
 */
std::string createMap(std::string_view key_type, std::string_view value_type);

// Integral Types
constexpr std::string_view BOOL{ "bool" };
constexpr std::string_view CHAR{ "char" };
constexpr std::string_view STRING{ "std::string" };
constexpr std::string_view INT{ "int" };
constexpr std::string_view UNSIGNED_INT{ "unsigned int" };
constexpr std::string_view LONG_INT{ "long int" };
constexpr std::string_view LONG_UNSIGNED_INT{ "long unsigned int" };
constexpr std::string_view FLOAT{ "float" };
constexpr std::string_view DOUBLE{ "double" };

// Eigen Types
constexpr std::string_view EIGEN_ISOMETRY_3D{ "Eigen::Isometry3d" };
// constexpr std::string_view EIGEN_MATRIX_XD{ "Eigen::MatrixXd" };
constexpr std::string_view EIGEN_VECTOR_XD{ "Eigen::VectorXd" };
// constexpr std::string_view EIGEN_MATRIX_2D{ "Eigen::Matrix2d" };
constexpr std::string_view EIGEN_VECTOR_2D{ "Eigen::Vector2d" };
// constexpr std::string_view EIGEN_MATRIX_3D{ "Eigen::Matrix3d" };
constexpr std::string_view EIGEN_VECTOR_3D{ "Eigen::Vector3d" };
}  // namespace property_type

namespace property_attribute
{
constexpr std::string_view TYPE{ "type" };
constexpr std::string_view DOC{ "doc" };
constexpr std::string_view REQUIRED{ "required" };
constexpr std::string_view DEFAULT{ "default" };
constexpr std::string_view ENUM{ "enum" };
constexpr std::string_view MINIMUM{ "minimum" };
constexpr std::string_view MAXIMUM{ "maximum" };
}  // namespace property_attribute

/**
 * @file property_tree.h
 * @brief Defines PropertyTree, a hierarchical structure for YAML-based
 *        configuration with metadata and validation support.
 */

/**
 * @class PropertyTree
 * @brief Represents a node in a hierarchical property tree.
 *
 * Each PropertyTree node may contain:
 *  - A YAML::Node value (scalar, sequence, or map)
 *  - A map of metadata attributes (YAML::Node) keyed by string
 *  - Nested child PropertyTree nodes
 *  - A list of custom validator functions
 */
class PropertyTree
{
public:
  /**
   * @brief Signature for custom validator callbacks.
   * @param node   The PropertyTree node being validated.
   * @throws std::runtime_error on validation failure.
   */
  using ValidatorFn = std::function<void(const PropertyTree&)>;

  /**
   * @brief Default constructor. Creates an empty tree node.
   */
  PropertyTree() = default;

  /**
   * @brief Given that *this* is purely a schema tree, merge in the
   *        user’s YAML config to populate all values (and apply defaults).
   * @param config  The user-supplied YAML::Node (possibly null).
   * @param allow_extra_properties  If false, “extra” keys in config will be flagged.
   */
  void mergeConfig(const YAML::Node& config, bool allow_extra_properties = false);

  /**
   * @brief Validate the tree using registered validators.
   *
   * Recursively invokes all validators on this node and its children.
   * Throws on the first error encountered.
   *
   * @param allow_extra_properties Indicate if extra properties are allowed
   */
  void validate(bool allow_extra_properties = false) const;

  /**
   * @brief Register a custom validator for this node.
   * @param fn  Callback to invoke during validation.
   */
  void addValidator(ValidatorFn fn);

  /**
   * @brief Access or create a child node by key.
   * @param key  Child identifier (string key).
   * @return Reference to the child PropertyTree.
   */
  PropertyTree& get(std::string_view key);

  /**
   * @brief Access a child node by key (const).
   * @param key  Child identifier.
   * @return Const reference to the child.
   * @throws std::out_of_range if key not found.
   */
  const PropertyTree& get(std::string_view key) const;

  /**
   * @brief Find a child node without creating it.
   * @param key  Child identifier.
   * @return Pointer to child or nullptr if not present.
   */
  const PropertyTree* find(std::string_view key) const;

  /**
   * @brief Set the YAML value of this node.
   * @param v  YAML::Node representing the new value.
   */
  void setValue(const YAML::Node& v);

  /**
   * @brief Retrieve the YAML value stored at this node.
   * @return Const reference to a YAML::Node.
   */
  const YAML::Node& getValue() const;

  /**
   * @brief Check if property value is null
   * @return True if required, otherwise false
   */
  bool isNull() const;

  /**
   * @brief Check if property is a container of child properties
   * @return True if container, otherwise false
   */
  bool isContainer() const;

  /**
   * @brief List all immediate child keys.
   * @return Vector of child key strings.
   */
  std::vector<std::string> keys() const;

  /**
   * @brief Set a metadata attribute (YAML node form).
   * @param name  Attribute name.
   * @param attr  YAML::Node value.
   */
  void setAttribute(std::string_view name, const YAML::Node& attr);

  /** @brief Convenience overload to set a string attribute. */
  void setAttribute(std::string_view name, std::string_view attr);
  void setAttribute(std::string_view name, const char* attr);
  /** @brief Set a boolean attribute. */
  void setAttribute(std::string_view name, bool attr);
  /** @brief Set an integer attribute. */
  void setAttribute(std::string_view name, int attr);
  /** @brief Set a double attribute. */
  void setAttribute(std::string_view name, double attr);

  /**
   * @brief Check if an attribute exists and is not null.
   * @param name  Attribute name.
   * @return True if present and non-null, false otherwise.
   */
  bool hasAttribute(std::string_view name) const;

  /**
   * @brief Retrieve an attribute value by name.
   * @param name  Attribute name.
   * @return Optional containing YAML::Node if found.
   */
  std::optional<YAML::Node> getAttribute(std::string_view name) const;

  /**
   * @brief List all metadata attribute keys.
   * @return Vector of attribute names.
   */
  std::vector<std::string> getAttributeKeys() const;

  /**
   * @brief Check if property is requried by checking for attribute and its value
   * @return True if required, otherwise false
   */
  bool isRequired() const;

  /**
   * @brief Create a PropertyTree from a YAML::Node.
   * @param node  Root YAML::Node to convert.
   * @return Populated PropertyTree hierarchy.
   */
  static PropertyTree fromYAML(const YAML::Node& node);

  /**
   * @brief Serialize this tree back into a YAML::Node.
   * @param exclude_attributes  If true, omit the attributes map.
   * @return YAML::Node representation of the tree.
   */
  YAML::Node toYAML(bool exclude_attributes = true) const;

private:
  YAML::Node value_;                             /**< Value stored at this node */
  YAML::Node follow_;                            /**< Follow stored at this node */
  std::map<std::string, YAML::Node> attributes_; /**< Metadata attributes */
  std::map<std::string, PropertyTree> children_; /**< Nested child nodes */
  std::vector<ValidatorFn> validators_;          /**< Validators to invoke */
};

/**
 * @brief Check if type is a sequence
 * @param type The type to check
 * @return If it is a sequence the underlying type is returned
 */
std::optional<std::string> isSequenceType(std::string_view type);

/**
 * @brief Validator: ensure 'required' attribute is present and non-null.
 * @param node  Node to validate.
 * @throws runtime_error if missing.
 */
void validateRequired(const PropertyTree& node);

/**
 * @brief Validator: enforce that node's value is in 'enum' list.
 * @param node  Node to validate.
 * @throws runtime_error if not found.
 */
void validateEnum(const PropertyTree& node);

/**
 * @brief Validtor: ensure node value is of type YAML::NodeType::Map
 * @param node Node to validate.
 * @throws runtime_error if not correct type.
 */
void validateMap(const PropertyTree& node);

/**
 * @brief Validtor: ensure node value is of type YAML::NodeType::Sequence
 * @param node Node to validate.
 * @throws runtime_error if not correct type.
 */
void validateSequence(const PropertyTree& node);

void validateCustomType(const PropertyTree& node);

/**
 * @brief Validate that the node’s value can be interpreted as the provided type.
 * @param node  PropertyTree node whose value to validate.
 * @throws std::runtime_error or YAML::BadConversion if the value is not a valid type.
 */
template <typename T>
void validateTypeCast(const PropertyTree& node)
{
  try
  {
    node.getValue().as<T>();
  }
  catch (const std::exception& e)
  {
    std::throw_with_nested(e);
  }
}

/**
 * @brief Validator: type cast and enforce 'minimum'/'maximum' range constraints.
 * @param node  Node to validate.
 * @throws runtime_error if out of range.
 */
template <typename T>
void validateTypeCastWithRange(const PropertyTree& node)
{
  // Get value
  const T val = [&]() {
    try
    {
      return node.getValue().as<T>();
    }
    catch (const std::exception& e)
    {
      std::throw_with_nested(e);
    }
  }();

  // If minimum attribute exist, validate
  auto min_attr = node.getAttribute(property_attribute::MINIMUM);
  if (min_attr)
  {
    const auto minv = min_attr->as<T>();
    if (val < minv)
      std::throw_with_nested(std::runtime_error("Property value " + std::to_string(val) + " is less than minimum " +
                                                std::to_string(minv)));
  }

  // If maximum attribute exist, validate
  auto max_attr = node.getAttribute(property_attribute::MAXIMUM);
  if (max_attr)
  {
    const auto maxv = max_attr->as<T>();
    if (val > maxv)
      std::throw_with_nested(std::runtime_error("Property value " + std::to_string(val) + " is greater than maximum " +
                                                std::to_string(maxv)));
  }
}

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PROPERTY_TREE_H
