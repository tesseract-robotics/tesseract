/**
 * @file property_tree.cpp
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

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registry.h>
#include <tesseract/common/yaml_extensions.h>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <ostream>
#include <regex>

const static std::string ATTRIBUTES_KEY{ "_attributes" };
const static std::string VALUE_KEY{ "_value" };
const static std::string EXTRA_KEY{ "_extra" };
const static std::string ONEOF_KEY{ "_oneof" };
const static std::string FOLLOW_KEY{ "follow" };

namespace tesseract::common
{
namespace property_type
{
std::string createList(std::string_view type, std::size_t length)
{
  if (length == 0)
    return std::string(type) + "[]";

  return std::string(type) + "[" + std::to_string(length) + "]";
}

std::string createMap(std::string_view key, std::string_view type)
{
  return "{" + std::string(key) + "," + std::string(type) + "}";
}
std::string createMap(std::string_view type) { return createMap(STRING, type); }
}  // namespace property_type

PropertyTree::PropertyTree(const PropertyTree& other)
  : value_(YAML::Clone(other.value_))
  , follow_(YAML::Clone(other.follow_))
  , children_(other.children_)
  , auto_validators_(other.auto_validators_)
  , validators_(other.validators_)
{
  if (other.oneof_ != nullptr)
    oneof_ = std::make_unique<PropertyTree>(*other.oneof_);

  // Deep-clone all attributes
  for (auto const& [k, node] : other.attributes_)
    attributes_[k] = YAML::Clone(node);
}

PropertyTree& PropertyTree::operator=(const PropertyTree& other)
{
  if (this == &other)
    return *this;

  // Clone the YAML values
  value_ = YAML::Clone(other.value_);
  follow_ = YAML::Clone(other.follow_);

  // Copy and clone attributes
  attributes_.clear();
  for (const auto& [k, node] : other.attributes_)
    attributes_[k] = YAML::Clone(node);

  // Copy children and validators
  children_ = other.children_;
  auto_validators_ = other.auto_validators_;
  validators_ = other.validators_;

  // Copy oneOf
  if (other.oneof_ != nullptr)
    oneof_ = std::make_unique<PropertyTree>(*other.oneof_);
  else
    oneof_ = nullptr;

  return *this;
}

void PropertyTree::mergeConfig(const YAML::Node& config, bool allow_extra_properties)
{
  // Handle oneOf nodes up front
  auto t = getAttribute(property_attribute::TYPE);
  if (t.has_value() && t->as<std::string>() == property_type::ONEOF)
  {
    if (!config || !config.IsMap())
      throw std::runtime_error("oneOf schema expects a YAML map");

    // Find exactly one branch whose schema-keys all appear in config
    std::string chosen;
    for (const auto& [branch_name, branch_schema] : children_)
    {
      bool matches = true;
      for (const auto& key : branch_schema.keys())
      {
        if (branch_schema.at(key).isRequired() && !config[key])
        {
          matches = false;
          break;
        }
      }

      if (matches)
      {
        if (!chosen.empty())
          throw std::runtime_error("oneOf: multiple branches match");

        chosen = branch_name;
      }
    }

    if (chosen.empty())
      throw std::runtime_error("oneOf: no branch matches the provided keys");

    // Store schema
    oneof_ = std::make_unique<PropertyTree>(*this);

    // Flatten: replace this node's children with the chosen branch's children
    PropertyTree schema_copy = at(chosen);
    *this = schema_copy;

    // Now call merge again
    mergeConfig(config, allow_extra_properties);
    return;
  }

  // Leaf-schema override for both maps and sequences:
  // If this schema node has no children, but the user provided
  // either a map or a sequence, just store it wholesale.
  if (children_.empty() && config && (config.IsMap() || config.IsSequence()))
  {
    value_ = config;
    return;
  }

  // Apply default if no config & not required
  auto def_it = attributes_.find(std::string(property_attribute::DEFAULT));
  bool required = isRequired();
  if ((!config || config.IsNull()) && def_it != attributes_.end() && !required)
    value_ = YAML::Clone(def_it->second);

  // Scalar or (now only non‐leaf) sequence override
  if (config && config.IsScalar())
    value_ = config;

  // Map: recurse into declared children
  if (config && config.IsMap())
  {
    // Fill declared children
    for (auto& [key, child_schema] : children_)
    {
      auto sub = config[key];
      child_schema.mergeConfig(sub, allow_extra_properties);
    }

    // Handle extras
    if (!allow_extra_properties)
    {
      for (auto it = config.begin(); it != config.end(); ++it)
      {
        const auto& key = it->first.as<std::string>();
        if (find(key) == nullptr)
        {
          auto& extra_node = (*this)[key];
          extra_node.setAttribute(EXTRA_KEY, YAML::Node(true));
          extra_node.mergeConfig(it->second, allow_extra_properties);
        }
      }
    }
  }

  // Sequence (non-leaf): apply wildcard or numeric schema
  else if (config && config.IsSequence())
  {
    PropertyTree elem_schema;
    auto* w = find("*");
    if (w != nullptr)
      elem_schema = *w;

    children_.clear();
    int idx = 0;
    for (const auto& elt : config)
    {
      std::string key = std::to_string(idx++);
      children_.emplace_back(std::move(key), elem_schema);
      children_.back().second.mergeConfig(elt, allow_extra_properties);
    }
  }
  // Otherwise leave value_ (possibly set by default) as-is.
}

std::vector<std::string> PropertyTree::validate(bool allow_extra_properties) const
{
  std::vector<std::string> errors;
  collectErrors(errors, "", allow_extra_properties);
  return errors;
}

void PropertyTree::collectErrors(std::vector<std::string>& errors,
                                 const std::string& path,
                                 bool allow_extra_properties) const
{
  // check if it is an extra property not found in schema
  if (!allow_extra_properties && hasAttribute(EXTRA_KEY))
    errors.push_back((path.empty() ? "(root)" : path) + ": property does not exist in schema");

  // recurse ALL children
  for (const auto& [key, child] : children_)
  {
    std::string child_path = path;
    if (!path.empty())
      child_path += ".";
    child_path += key;
    child.collectErrors(errors, child_path, allow_extra_properties);
  }

  // if not required and null skip validators
  if (!isRequired() && isNull())
    return;

  std::string my_path = path.empty() ? "(root)" : path;

  // run auto-validators derived from attributes
  for (const auto& vfn : auto_validators_)
    vfn(*this, my_path, errors);

  // run user-added validators
  for (const auto& vfn : validators_)
    vfn(*this, my_path, errors);
}

void PropertyTree::addValidator(ValidatorFn fn) { validators_.push_back(std::move(fn)); }

PropertyTree& PropertyTree::operator[](std::string_view key)
{
  for (auto& [k, v] : children_)
  {
    if (k == key)
      return v;
  }
  children_.emplace_back(std::string(key), PropertyTree{});
  return children_.back().second;
}

PropertyTree& PropertyTree::at(std::string_view key)
{
  for (auto& [k, v] : children_)
  {
    if (k == key)
      return v;
  }
  throw std::out_of_range("PropertyTree::at: key not found -> " + std::string(key));
}

const PropertyTree& PropertyTree::at(std::string_view key) const
{
  for (const auto& [k, v] : children_)
  {
    if (k == key)
      return v;
  }
  throw std::out_of_range("PropertyTree::at: key not found -> " + std::string(key));
}

const PropertyTree* PropertyTree::find(std::string_view key) const
{
  for (const auto& [k, v] : children_)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

PropertyTree* PropertyTree::find(std::string_view key)
{
  for (auto& [k, v] : children_)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

void PropertyTree::setValue(const YAML::Node& v) { value_ = v; }
const YAML::Node& PropertyTree::getValue() const { return value_; }

bool PropertyTree::isNull() const { return value_.IsNull(); }

bool PropertyTree::isContainer() const { return !children_.empty(); }

std::size_t PropertyTree::size() const { return children_.size(); }

bool PropertyTree::empty() const { return children_.empty(); }

std::vector<std::string> PropertyTree::keys() const
{
  std::vector<std::string> result;
  result.reserve(children_.size());
  for (const auto& [k, v] : children_)
    result.push_back(k);
  return result;
}

void PropertyTree::setAttribute(std::string_view name, const YAML::Node& attr)
{
  attributes_[std::string(name)] = attr;

  // Rebuild auto-validators whenever a relevant attribute changes
  if (name == property_attribute::REQUIRED || name == property_attribute::ENUM || name == property_attribute::TYPE)
    rebuildAutoValidators();
}

void PropertyTree::rebuildAutoValidators()
{
  // Clear and rebuild all attribute-derived validators from scratch
  // This prevents accumulation when setAttribute is called multiple times.
  auto_validators_.clear();

  if (hasAttribute(property_attribute::REQUIRED))
    auto_validators_.emplace_back(validateRequired);

  if (hasAttribute(property_attribute::ENUM))
    auto_validators_.emplace_back(validateEnum);

  auto type_attr = getAttribute(property_attribute::TYPE);
  if (type_attr.has_value())
  {
    const auto str_type = type_attr->as<std::string>();

    std::optional<std::pair<std::string, std::size_t>> is_sequence = isSequenceType(str_type);
    if (is_sequence.has_value())
      auto_validators_.emplace_back([length = is_sequence.value().second](const PropertyTree& node,
                                                                          const std::string& path,
                                                                          std::vector<std::string>& errors) {
        validateSequence(node, length, path, errors);
      });

    std::optional<std::pair<std::string, std::string>> is_map = isMapType(str_type);
    if (is_map.has_value())
      auto_validators_.emplace_back(validateMap);

    if (str_type == property_type::CONTAINER)
      auto_validators_.emplace_back(validateContainer);
    else if (str_type == property_type::STRING)
      auto_validators_.emplace_back(validateTypeCast<std::string>);
    else if (str_type == property_type::BOOL)
      auto_validators_.emplace_back(validateTypeCast<bool>);
    else if (str_type == property_type::CHAR)
      auto_validators_.emplace_back(validateTypeCast<char>);
    else if (str_type == property_type::FLOAT)
      auto_validators_.emplace_back(validateTypeCastWithRange<float>);
    else if (str_type == property_type::DOUBLE)
      auto_validators_.emplace_back(validateTypeCastWithRange<double>);
    else if (str_type == property_type::INT)
      auto_validators_.emplace_back(validateTypeCastWithRange<int>);
    else if (str_type == property_type::UNSIGNED_INT)
      auto_validators_.emplace_back(validateTypeCastWithRange<unsigned int>);
    else if (str_type == property_type::LONG_INT)
      auto_validators_.emplace_back(validateTypeCastWithRange<long int>);
    else if (str_type == property_type::LONG_UNSIGNED_INT)
      auto_validators_.emplace_back(validateTypeCastWithRange<long unsigned int>);
    else if (str_type == property_type::EIGEN_ISOMETRY_3D)
      auto_validators_.emplace_back(validateTypeCast<Eigen::Isometry3d>);
    // else if (str_type == property_type::EIGEN_MATRIX_2D)
    //   auto_validators_.emplace_back(validateTypeCast<Eigen::MatrixXd>);
    else if (str_type == property_type::EIGEN_VECTOR_XD)
    {
      auto_validators_.emplace_back([](const PropertyTree& node,
                                       const std::string& path,
                                       std::vector<std::string>& errors) { validateSequence(node, 0, path, errors); });
      auto_validators_.emplace_back(validateTypeCast<Eigen::VectorXd>);
    }
    // else if (str_type == property_type::EIGEN_MATRIX_2D)
    //   auto_validators_.emplace_back(validateTypeCast<Eigen::Matrix2d>);
    else if (str_type == property_type::EIGEN_VECTOR_2D)
    {
      auto_validators_.emplace_back([](const PropertyTree& node,
                                       const std::string& path,
                                       std::vector<std::string>& errors) { validateSequence(node, 2, path, errors); });
      auto_validators_.emplace_back(validateTypeCast<Eigen::Vector2d>);
    }
    // else if (str_type == property_type::EIGEN_MATRIX_3D)
    //   auto_validators_.emplace_back(validateTypeCast<Eigen::Matrix3d>);
    else if (str_type == property_type::EIGEN_VECTOR_3D)
    {
      auto_validators_.emplace_back([](const PropertyTree& node,
                                       const std::string& path,
                                       std::vector<std::string>& errors) { validateSequence(node, 3, path, errors); });
      auto_validators_.emplace_back(validateTypeCast<Eigen::Vector3d>);
    }
  }
}

void PropertyTree::setAttribute(std::string_view name, std::string_view attr)
{
  setAttribute(name, YAML::Node(std::string(attr)));
}

void PropertyTree::setAttribute(std::string_view name, const char* attr)
{
  setAttribute(name, YAML::Node(std::string(attr)));
}

void PropertyTree::setAttribute(std::string_view name, bool attr) { setAttribute(name, YAML::Node(attr)); }

void PropertyTree::setAttribute(std::string_view name, int attr) { setAttribute(name, YAML::Node(attr)); }

void PropertyTree::setAttribute(std::string_view name, double attr) { setAttribute(name, YAML::Node(attr)); }

void PropertyTree::setAttribute(std::string_view name, const std::vector<std::string>& attr)
{
  setAttribute(name, YAML::Node(attr));
}

bool PropertyTree::hasAttribute(std::string_view name) const
{
  auto it = attributes_.find(std::string(name));
  return (it != attributes_.end() && it->second && !it->second.IsNull());
}

std::optional<YAML::Node> PropertyTree::getAttribute(std::string_view name) const
{
  auto it = attributes_.find(std::string(name));
  if (it != attributes_.end())
    return it->second;
  return std::nullopt;
}

std::vector<std::string> PropertyTree::getAttributeKeys() const
{
  std::vector<std::string> res;
  res.reserve(attributes_.size());
  for (const auto& pair : attributes_)
    res.push_back(pair.first);
  return res;
}

bool PropertyTree::isRequired() const
{
  std::optional<YAML::Node> required = getAttribute(property_attribute::REQUIRED);
  if (!required.has_value())
    return false;

  return required.value().as<bool>();
}

PropertyTree PropertyTree::fromYAML(const YAML::Node& node)
{
  // Handle 'follow' directive: load external YAML or schema file
  if (node.IsMap() && node[std::string(FOLLOW_KEY)] && node[std::string(FOLLOW_KEY)].IsScalar())
  {
    if (node.size() > 1)
      throw std::runtime_error("'follow' cannot be mixed with other entries");

    try
    {
      auto key = node[std::string(FOLLOW_KEY)].as<std::string>();
      auto registry = SchemaRegistry::instance();
      return registry->contains(key) ? registry->get(key) : SchemaRegistry::loadFile(key);
    }
    catch (const std::exception&)
    {
      throw;  // rethrow the original exception
    }
  }

  PropertyTree tree;
  if (node.IsMap())
  {
    // extract attributes if it exists
    if (node[std::string(ATTRIBUTES_KEY)] && node[std::string(ATTRIBUTES_KEY)].IsMap())
    {
      for (const auto& it : node[std::string(ATTRIBUTES_KEY)])
      {
        const auto key = it.first.as<std::string>();
        tree.attributes_[key] = it.second;
      }
    }

    // extract oneof if it exist
    if (node[std::string(ONEOF_KEY)] && node[std::string(ONEOF_KEY)].IsMap())
      tree.oneof_ = std::make_unique<PropertyTree>(fromYAML(node[std::string(ONEOF_KEY)]));

    // extract the value if it exists (for leaves with attributes)
    if (node[std::string(VALUE_KEY)])
      tree.value_ = node[std::string(VALUE_KEY)];

    // extract children
    for (const auto& it : node)
    {
      const auto key = it.first.as<std::string>();
      if (key == std::string(ATTRIBUTES_KEY) || key == std::string(ONEOF_KEY) || key == std::string(VALUE_KEY))
        continue;

      tree.children_.emplace_back(key, fromYAML(it.second));
    }
  }
  else if (node.IsSequence())
  {
    tree.value_ = node;
    int idx = 0;
    for (const auto& it : node)
      tree.children_.emplace_back(std::to_string(idx++), fromYAML(it));
  }
  else
  {
    // Scalar value
    tree.value_ = node;
  }
  return tree;
}

YAML::Node PropertyTree::toYAML(bool exclude_attributes) const
{
  // pure leaf without attributes or children: emit scalar/sequence directly
  if (attributes_.empty() && children_.empty())
    return value_;

  // pure leaf with attributes excluded and no children: emit scalar/sequence directly
  if (exclude_attributes && children_.empty())
    return value_;

  // Always emit a mapping if attributes exist or children exist
  YAML::Node node(YAML::NodeType::Map);
  // emit attributes first
  if (!exclude_attributes && !attributes_.empty())
  {
    YAML::Node attr_node(YAML::NodeType::Map);
    for (const auto& pair : attributes_)
      attr_node[pair.first] = pair.second;

    node[std::string(ATTRIBUTES_KEY)] = attr_node;
  }

  // emit children
  for (const auto& [key, child] : children_)
  {
    // If the property is not required and is null then skip when excluding attributes
    if (exclude_attributes && !child.isRequired() && child.isNull())
      continue;

    node[key] = child.toYAML(exclude_attributes);
  }

  // emit oneof
  if (!exclude_attributes && oneof_ != nullptr)
    node[std::string(ONEOF_KEY)] = oneof_->toYAML(exclude_attributes);

  // if leaf (no children) but value present, emit under 'value'
  if (children_.empty() && value_)
    node[std::string(VALUE_KEY)] = value_;

  return node;
}

PropertyTree::operator bool() const noexcept { return (!children_.empty() || !value_.IsNull()); }

std::ostream& operator<<(std::ostream& os, const PropertyTree& tree)
{
  os << tree.toYAML(/*exclude_attributes=*/false);
  return os;
}

// ——— PropertyTreeBuilder ———

PropertyTreeBuilder::PropertyTreeBuilder() { stack_.push_back(&root_); }

PropertyTree& PropertyTreeBuilder::current() { return *stack_.back(); }

PropertyTreeBuilder& PropertyTreeBuilder::container(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::string(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::STRING);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::character(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::CHAR);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::boolean(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::BOOL);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::integer(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::INT);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::unsignedInt(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::UNSIGNED_INT);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::longInt(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::LONG_INT);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::longUnsignedInt(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::LONG_UNSIGNED_INT);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::floatNum(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::FLOAT);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::doubleNum(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::DOUBLE);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::customType(std::string_view name, std::string_view type_str)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, type_str);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::doc(std::string_view text)
{
  current().setAttribute(property_attribute::DOC, text);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::required()
{
  current().setAttribute(property_attribute::REQUIRED, true);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(bool val)
{
  current().setAttribute(property_attribute::DEFAULT, YAML::Node(val));
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(int val)
{
  current().setAttribute(property_attribute::DEFAULT, YAML::Node(val));
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(double val)
{
  current().setAttribute(property_attribute::DEFAULT, YAML::Node(val));
  return *this;
}
PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(const char* val) { return defaultVal(std::string_view(val)); }
PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(std::string_view val)
{
  current().setAttribute(property_attribute::DEFAULT, YAML::Node(std::string(val)));
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::defaultVal(const YAML::Node& val)
{
  current().setAttribute(property_attribute::DEFAULT, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::enumValues(const std::vector<std::string>& values)
{
  current().setAttribute(property_attribute::ENUM, values);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::minimum(int val)
{
  current().setAttribute(property_attribute::MINIMUM, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::minimum(double val)
{
  current().setAttribute(property_attribute::MINIMUM, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::maximum(int val)
{
  current().setAttribute(property_attribute::MAXIMUM, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::maximum(double val)
{
  current().setAttribute(property_attribute::MAXIMUM, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::label(std::string_view text)
{
  current().setAttribute(property_attribute::LABEL, text);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::placeholder(std::string_view text)
{
  current().setAttribute(property_attribute::PLACEHOLDER, text);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::group(std::string_view text)
{
  current().setAttribute(property_attribute::GROUP, text);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::readOnly(bool val)
{
  current().setAttribute(property_attribute::READ_ONLY, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::hidden(bool val)
{
  current().setAttribute(property_attribute::HIDDEN, val);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::validator(PropertyTree::ValidatorFn fn)
{
  current().addValidator(std::move(fn));
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::attribute(std::string_view name, const YAML::Node& value)
{
  current().setAttribute(name, value);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::attribute(std::string_view name, std::string_view value)
{
  current().setAttribute(name, value);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::done()
{
  if (stack_.size() <= 1)
    throw std::runtime_error("PropertyTreeBuilder::done() called at root level");
  stack_.pop_back();
  return *this;
}

PropertyTree PropertyTreeBuilder::build()
{
  if (stack_.size() != 1)
    throw std::runtime_error("PropertyTreeBuilder::build() called with unclosed scopes — missing done() calls");
  return std::move(root_);
}

std::optional<std::pair<std::string, std::size_t>> isSequenceType(std::string_view type)
{
  static std::once_flag seq_flag;
  static std::unique_ptr<std::regex> re;
  std::call_once(seq_flag, []() { re = std::make_unique<std::regex>(R"(^([^\[\]]+)\[(\d*)\]$)"); });

  std::string s{ type };

  std::smatch m;
  if (std::regex_match(s, m, *re))
  {
    std::string base_type = m[1].str();
    std::string length_str = m[2].str();
    std::size_t length{ 0 };
    if (!length_str.empty())
    {
      if (!toNumeric<std::size_t>(length_str, length))
        throw std::runtime_error("Invalid fixed size sequence definition");
    }

    return std::make_pair(base_type, length);
  }

  return std::nullopt;
}

std::optional<std::pair<std::string, std::string>> isMapType(std::string_view type)
{
  static std::once_flag map_flag;
  static std::unique_ptr<std::regex> re;
  std::call_once(map_flag, []() { re = std::make_unique<std::regex>(R"(^\{([^,]+),([^}]+)\}$)"); });

  std::string s{ type };
  std::smatch m;

  // m[0] is the full match, m[1] is the first capture, m[2] the second
  if (std::regex_match(s, m, *re))
    return std::make_pair(m[1].str(), m[2].str());

  return std::nullopt;
}

void validateRequired(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  auto req_attr = node.getAttribute(property_attribute::REQUIRED);
  if (req_attr && req_attr->as<bool>())
  {
    // if leaf node with no value or null
    if (!node.isContainer() && node.isNull())
      errors.push_back(path + ": required property missing or null");
  }
}

void validateEnum(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  auto enum_attr = node.getAttribute(property_attribute::ENUM);
  if (enum_attr.has_value() && enum_attr->IsSequence())
  {
    const auto val = node.getValue().as<std::string>();
    for (const auto& v : enum_attr.value())
    {
      if (v.as<std::string>() == val)
        return;
    }
    errors.push_back(path + ": value '" + val + "' not in enum list");
  }
}

void validateMap(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  if (!node.getValue().IsMap())
    errors.push_back(path + ": value is not of type YAML::NodeType::Map");
}

void validateSequence(const PropertyTree& node,
                      std::size_t length,
                      const std::string& path,
                      std::vector<std::string>& errors)
{
  if (!node.getValue().IsSequence())
  {
    errors.push_back(path + ": value is not of type YAML::NodeType::Sequence");
    return;
  }

  if (length != 0 && node.getValue().size() != length)
    errors.push_back(path + ": sequence length " + std::to_string(node.getValue().size()) +
                     " does not match expected " + std::to_string(length));
}

void validateContainer(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  if (!node.isContainer())
    errors.push_back(path + ": property is not a container");

  if (!node.isNull())
    errors.push_back(path + ": property is a container but value is not null");
}

void validateCustomType(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  const auto type_attr = node.getAttribute(property_attribute::TYPE);
  if (!type_attr.has_value())
  {
    errors.push_back(path + ": custom type validator was added but type attribute does not exist");
    return;
  }

  const auto type_str = type_attr.value().as<std::string>();
  std::optional<std::pair<std::string, std::size_t>> is_sequence = isSequenceType(type_str);

  auto registry = SchemaRegistry::instance();
  if (!is_sequence.has_value())
  {
    if (!registry->contains(type_str))
    {
      errors.push_back(path + ": no schema registry entry found for key: " + type_str);
      return;
    }

    PropertyTree schema = registry->get(type_str);
    schema.mergeConfig(node.getValue());
    auto sub_errors = schema.validate();
    errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
  }
  else
  {
    if (!registry->contains(is_sequence.value().first))
    {
      errors.push_back(path + ": no schema registry entry found for key: " + is_sequence.value().first);
      return;
    }

    const YAML::Node& sequence = node.getValue();
    PropertyTree schema = registry->get(is_sequence.value().first);
    std::size_t idx = 0;
    for (auto it = sequence.begin(); it != sequence.end(); ++it, ++idx)
    {
      PropertyTree copy_schema(schema);
      copy_schema.mergeConfig(*it);
      std::string elem_path = path + "[" + std::to_string(idx) + "]";
      // Collect errors with element path context
      std::vector<std::string> sub_errors;
      copy_schema.collectErrors(sub_errors, elem_path, false);
      errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
    }
  }
}

}  // namespace tesseract::common
