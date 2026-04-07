/**
 * @file property_tree.h
 * @brief This is a property tree class
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
#ifndef TESSERACT_COMMON_PROPERTY_TREE_H
#define TESSERACT_COMMON_PROPERTY_TREE_H

#include <string>
#include <string_view>
#include <map>
#include <vector>
#include <optional>
#include <functional>
#include <iosfwd>
#include <yaml-cpp/yaml.h>

namespace tesseract::common
{
namespace property_type
{
/**
 * @brief A utility for constructing the vector<type>
 * @param type The type assoicated with the list
 * @param length The length if fixed size
 * @return The string representation of the vector<type>, aka. type[] and type[length] for fixed size
 */
std::string createList(std::string_view type, std::size_t length = 0);

/**
 * @brief A utility for constructing the map<std::string, type>
 * @param type The value type assoicated with the map
 * @return The string representation of the map<std::string, type>, aka. {string,string}
 */
std::string createMap(std::string_view type);

/**
 * @brief A utility for constructing the map<key, type>
 * @param type The value type assoicated with the map
 * @return The string representation of the map<key, type>, aka. {string, string} or {string[2], string}
 */
std::string createMap(std::string_view key, std::string_view type);

// Integral Types
constexpr std::string_view BOOL{ "bool" };
constexpr std::string_view CHAR{ "char" };
constexpr std::string_view STRING{ "string" };
constexpr std::string_view INT32{ "int32" };
constexpr std::string_view UINT32{ "uint32" };
constexpr std::string_view INT64{ "int64" };
constexpr std::string_view UINT64{ "uint64" };
constexpr std::string_view FLOAT32{ "float32" };
constexpr std::string_view FLOAT64{ "float64" };

// Container of properties
constexpr std::string_view CONTAINER{ "container" };
constexpr std::string_view ONEOF{ "oneOf" };

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
constexpr std::string_view ACCEPTS_DERIVED_TYPES{ "accepts_derived_types" }; /**< Allow derived types for custom type
                                                                                  validation */

// GUI metadata attributes
constexpr std::string_view LABEL{ "label" };             /**< Display name for the property (e.g. "Format Result") */
constexpr std::string_view PLACEHOLDER{ "placeholder" }; /**< Hint text shown when the field is empty */
constexpr std::string_view GROUP{ "group" };             /**< Category/section for organizing properties in the GUI */
constexpr std::string_view READ_ONLY{ "read_only" };     /**< If true, the property is not user-editable */
constexpr std::string_view HIDDEN{ "hidden" };           /**< If true, the property is hidden from the GUI */
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
   * @param path   Dot-separated path to this node (for error messages).
   * @param errors Output vector to append error messages to.
   */
  using ValidatorFn =
      std::function<void(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)>;

  /**
   * @brief Default constructor. Creates an empty tree node.
   */
  PropertyTree() = default;
  ~PropertyTree() = default;

  /** @brief Deep-copy constructor -- clones all YAML::Nodes and subtrees */
  PropertyTree(const PropertyTree& other);
  /** @brief Deep-copy assignment operator */
  PropertyTree& operator=(const PropertyTree& other);

  /** @brief Default move constructor/assignment are fine */
  PropertyTree(PropertyTree&&) noexcept = default;
  PropertyTree& operator=(PropertyTree&&) noexcept = default;

  /**
   * @brief Given that *this* is purely a schema tree, merge in the
   *        user's YAML config to populate all values (and apply defaults).
   * @param config  The user-supplied YAML::Node (possibly null).
   * @param allow_extra_properties  If false, "extra" keys in config will be flagged.
   */
  void mergeConfig(const YAML::Node& config, bool allow_extra_properties = false);

  /**
   * @brief Validate the tree, collecting all errors.
   *
   * Recursively invokes all validators on this node and its children,
   * collecting every error rather than stopping at the first one.
   *
   * @param allow_extra_properties Indicate if extra properties are allowed.
   * @return Vector of error strings (empty on success). Each entry
   *         includes the dot-separated path to the offending node.
   */
  [[nodiscard]] std::vector<std::string> validate(bool allow_extra_properties = false) const;

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
  PropertyTree& operator[](std::string_view key);

  /**
   * @brief Access a child node by key.
   * @param key  Child identifier.
   * @return Reference to the child.
   * @throws std::out_of_range if key not found.
   */
  PropertyTree& at(std::string_view key);

  /**
   * @brief Access a child node by key (const).
   * @param key  Child identifier.
   * @return Const reference to the child.
   * @throws std::out_of_range if key not found.
   */
  const PropertyTree& at(std::string_view key) const;

  /**
   * @brief Find a child node without creating it.
   * @param key  Child identifier.
   * @return Pointer to child or nullptr if not present.
   */
  const PropertyTree* find(std::string_view key) const;

  /**
   * @brief Find a child node without creating it (mutable).
   * @param key  Child identifier.
   * @return Pointer to child or nullptr if not present.
   */
  PropertyTree* find(std::string_view key);

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
   * @brief Retrieve the YAML value casted to the provided type
   * @return The casted value
   */
  template <typename T>
  T as() const
  {
    return value_.as<T>();
  }

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
  /** @brief Set a vector of strings attribute. */
  void setAttribute(std::string_view name, const std::vector<std::string>& attr);

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

  /** @brief Indicate if defined, meaning it has children or a value */
  explicit operator bool() const noexcept;

  /** @brief Stream output -- serializes toYAML() to the stream. */
  friend std::ostream& operator<<(std::ostream& os, const PropertyTree& tree);

  /**
   * @brief Return number of immediate children.
   * @return Child count.
   */
  std::size_t size() const;

  /**
   * @brief Check if this node has no children.
   * @return True if no children.
   */
  bool empty() const;

private:
  friend void validateCustomType(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

  /** @brief Rebuild auto-validators from current attributes (type, required, enum). */
  void rebuildAutoValidators();

  /** @brief Recursive implementation of validate with path tracking. */
  void collectErrors(std::vector<std::string>& errors, const std::string& path, bool allow_extra_properties) const;

  YAML::Node value_;                                           /**< Value stored at this node */
  YAML::Node follow_;                                          /**< Follow stored at this node */
  std::map<std::string, YAML::Node> attributes_;               /**< Metadata attributes */
  std::vector<std::pair<std::string, PropertyTree>> children_; /**< Nested child nodes (insertion-ordered) */
  std::vector<ValidatorFn> auto_validators_; /**< Validators derived from attributes (rebuilt, not user-added) */
  std::vector<ValidatorFn> validators_;      /**< User-added validators to invoke */
  std::unique_ptr<PropertyTree> oneof_;      /**< Store the property content on merge */
};

/**
 * @class PropertyTreeBuilder
 * @brief Fluent builder for constructing PropertyTree schemas with minimal boilerplate.
 *
 * Usage:
 * @code
 * auto schema = PropertyTreeBuilder()
 *   .container("config")
 *     .doc("Main config").required()
 *     .boolean("conditional").doc("Enable conditional").defaultVal(true).done()
 *     .string("program").doc("The program").required().done()
 *   .done()
 *   .build();
 * @endcode
 *
 * Schema composition example (reuse base schemas):
 * @code
 * auto base = PropertyTreeBuilder()
 *   .attribute(TYPE, CONTAINER)
 *   .string("name").required().done()
 *   .build();
 *
 * auto extended = PropertyTreeBuilder()
 *   .attribute(TYPE, CONTAINER)
 *   .compose(base)              // copies name field
 *   .integer("priority").done()
 *   .build();
 * @endcode
 *
 * Inline oneOf example (shared fields with mutually exclusive parts):
 * @code
 * auto schema = PropertyTreeBuilder()
 *   .string(\"name\").required().done()
 *   .beginOneOf()
 *     .container(\"option_a\")
 *       .string(\"field_a\").required().done()
 *     .done()
 *     .container(\"option_b\")
 *       .integer(\"field_b\").required().done()
 *     .done()
 *   .endOneOf()
 *   .build();
 * @endcode
 *
 * Type-creating methods (container, string, boolean, etc.) create a child node,
 * set its type, and push it as the current scope. Attribute setters (doc, required,
 * defaultVal, etc.) apply to the current scope. done() pops back to the parent.
 * Use compose() to copy all children from an existing PropertyTree into the current scope.
 * Use beginOneOf()/endOneOf() to define mutually exclusive branches inline within
 * a container — shared fields are defined outside the oneOf block.
 */
class PropertyTreeBuilder
{
public:
  PropertyTreeBuilder();

  /** @name Type-creating methods -- create a typed child and descend into it. */
  ///@{
  PropertyTreeBuilder& container(std::string_view name);
  PropertyTreeBuilder& string(std::string_view name);
  PropertyTreeBuilder& character(std::string_view name);
  PropertyTreeBuilder& boolean(std::string_view name);
  PropertyTreeBuilder& int32(std::string_view name);
  PropertyTreeBuilder& uint32(std::string_view name);
  PropertyTreeBuilder& int64(std::string_view name);
  PropertyTreeBuilder& uint64(std::string_view name);
  PropertyTreeBuilder& float32(std::string_view name);
  PropertyTreeBuilder& float64(std::string_view name);
  PropertyTreeBuilder& eigenIsometry3d(std::string_view name);
  PropertyTreeBuilder& eigenVectorXd(std::string_view name);
  PropertyTreeBuilder& eigenVector2d(std::string_view name);
  PropertyTreeBuilder& eigenVector3d(std::string_view name);
  PropertyTreeBuilder& customType(std::string_view name, std::string_view type_str);

  /**
   * @brief Begin an inline oneOf group inside a container.
   *
   * Define mutually exclusive branches as container children within this group.
   * During mergeConfig the parent's full config is used for branch selection, and
   * the chosen branch's children are hoisted into the parent.  Close with endOneOf().
   */
  PropertyTreeBuilder& beginOneOf();

  /** @brief Close an inline oneOf group opened by beginOneOf(). */
  PropertyTreeBuilder& endOneOf();
  ///@}

  /** @name Attribute setters -- apply to the current node. */
  ///@{
  PropertyTreeBuilder& doc(std::string_view text);
  PropertyTreeBuilder& required();
  PropertyTreeBuilder& defaultVal(bool val);
  PropertyTreeBuilder& defaultVal(int val);
  PropertyTreeBuilder& defaultVal(double val);
  PropertyTreeBuilder& defaultVal(const char* val);
  PropertyTreeBuilder& defaultVal(std::string_view val);
  PropertyTreeBuilder& defaultVal(const YAML::Node& val);
  PropertyTreeBuilder& enumValues(const std::vector<std::string>& values);
  PropertyTreeBuilder& minimum(int val);
  PropertyTreeBuilder& minimum(double val);
  PropertyTreeBuilder& maximum(int val);
  PropertyTreeBuilder& maximum(double val);
  PropertyTreeBuilder& label(std::string_view text);
  PropertyTreeBuilder& placeholder(std::string_view text);
  PropertyTreeBuilder& group(std::string_view text);
  PropertyTreeBuilder& readOnly(bool val = true);
  PropertyTreeBuilder& hidden(bool val = true);
  PropertyTreeBuilder& validator(PropertyTree::ValidatorFn fn);
  PropertyTreeBuilder& attribute(std::string_view name, const YAML::Node& value);
  PropertyTreeBuilder& attribute(std::string_view name, std::string_view value);

  /**
   * @brief Mark that this field accepts the base type and any registered derived types.
   * Only applicable when the type attribute is a registered custom type.
   * @return Reference to this builder for method chaining.
   */
  PropertyTreeBuilder& acceptsDerivedTypes();
  ///@}

  /** @name Plugin helpers -- self-closing convenience methods for PluginInfoContainer schemas. */
  ///@{
  /**
   * @brief Create a PluginInfoContainer child: { default (string), plugins (Map of derived types) }.
   * Self-closing — does not push a scope; no done() call needed.
   * @param name                Child property name (e.g. "discrete_plugins").
   * @param factory_base_type   Fully-qualified factory base class (e.g.
   *                            "tesseract::collision::DiscreteContactManagerFactory").
   */
  PropertyTreeBuilder& pluginContainer(std::string_view name, std::string_view factory_base_type);

  /**
   * @brief Create a Map[string, PluginInfoContainer] child.
   *
   * Registers an intermediate PluginInfoContainer schema under the key
   * "<factory_base_type>::PluginInfoContainer" (idempotent) and creates a map
   * type that references it.  Self-closing — no done() call needed.
   * @param name                Child property name (e.g. "fwd_kin_plugins").
   * @param factory_base_type   Fully-qualified factory base class.
   */
  PropertyTreeBuilder& pluginContainerMap(std::string_view name, std::string_view factory_base_type);
  ///@}

  /**
   * @brief Copy all children from another PropertyTree into the current scope.
   *
   * This allows composing schemas together. The source tree's immediate children
   * (with their full subtrees, attributes, and validators) are appended to the
   * current builder scope.  The source tree's own root-level attributes are ignored;
   * only its children are composed.
   *
   * @param source The PropertyTree whose children should be copied into this scope.
   * @return Reference to this builder for method chaining.
   */
  PropertyTreeBuilder& compose(const PropertyTree& source);

  /** @brief Pop current node, return to parent scope. */
  PropertyTreeBuilder& done();

  /** @brief Finalize and return the constructed PropertyTree. */
  PropertyTree build();

private:
  PropertyTree& current();
  PropertyTree root_;
  std::vector<PropertyTree*> stack_;
};

/**
 * @brief Build a PluginInfoContainer schema for the given factory base type.
 * @param factory_base_type Fully-qualified factory base class name.
 * @return A PropertyTree with { default (string), plugins (Map of derived types) }.
 */
PropertyTree makePluginInfoContainerSchema(std::string_view factory_base_type);

/**
 * @brief Check if type is a sequence
 * @param type The type to check
 * @return If it is a sequence, the underlying type is returned and size
 */
std::optional<std::pair<std::string, std::size_t>> isSequenceType(std::string_view type);

/**
 * @brief Check if type is a map
 * @param type The type to check
 * @return If it is a map, the underlying pair<key,type> is returned
 */
std::optional<std::pair<std::string, std::string>> isMapType(std::string_view type);

/**
 * @brief Validator: ensure 'required' attribute is present and non-null.
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateRequired(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

/**
 * @brief Validator: enforce that node's value is in 'enum' list.
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateEnum(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

/**
 * @brief Validator: ensure node value is of type YAML::NodeType::Map
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateMap(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

/**
 * @brief Validator: ensure node value is of type YAML::NodeType::Sequence
 * @param node   Node to validate.
 * @param length The length if fixed size. If zero, it is considered dynamic size sequence.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateSequence(const PropertyTree& node,
                      std::size_t length,
                      const std::string& path,
                      std::vector<std::string>& errors);

/**
 * @brief Validator: ensure property is a container of child properties.
 * The property should have children and the value should be null.
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateContainer(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

/**
 * @brief Validator: Retrieve schema for the custom type and run its validators.
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
void validateCustomType(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors);

/**
 * @brief Validator: Validate a plugin info structure (class + config) for derived types.
 * @param node              Node to validate (should contain 'class' and 'config' fields).
 * @param base_type         The base type name to check against.
 * @param path              Dot-separated path for error messages.
 * @param errors            Output vector to append errors to.
 */
void validatePluginInfo(const PropertyTree& node,
                        const std::string& base_type,
                        const std::string& path,
                        std::vector<std::string>& errors);

/**
 * @brief Validate that the node's value can be interpreted as the provided type.
 * @param node   PropertyTree node whose value to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
template <typename T>
void validateTypeCast(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  try
  {
    node.getValue().as<T>();
  }
  catch (const std::exception& e)
  {
    errors.push_back(path + ": " + e.what());
  }
}

/**
 * @brief Validator: type cast and enforce 'minimum'/'maximum' range constraints.
 * @param node   Node to validate.
 * @param path   Dot-separated path for error messages.
 * @param errors Output vector to append errors to.
 */
template <typename T>
void validateTypeCastWithRange(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  T val;
  try
  {
    val = node.getValue().as<T>();
  }
  catch (const std::exception& e)
  {
    errors.push_back(path + ": " + e.what());
    return;  // can't check range if conversion failed
  }

  auto min_attr = node.getAttribute(property_attribute::MINIMUM);
  if (min_attr)
  {
    const auto minv = min_attr->as<T>();
    if (val < minv)
      errors.push_back(path + ": value " + std::to_string(val) + " is less than minimum " + std::to_string(minv));
  }

  auto max_attr = node.getAttribute(property_attribute::MAXIMUM);
  if (max_attr)
  {
    const auto maxv = max_attr->as<T>();
    if (val > maxv)
      errors.push_back(path + ": value " + std::to_string(val) + " is greater than maximum " + std::to_string(maxv));
  }
}

}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_PROPERTY_TREE_H
