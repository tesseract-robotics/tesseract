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
#include <iterator>
#include <memory>
#include <mutex>
#include <ostream>
#include <regex>
#include <set>

const static std::string ATTRIBUTES_KEY{ "_attributes" };
const static std::string VALUE_KEY{ "_value" };
const static std::string EXTRA_KEY{ "_extra" };
const static std::string FOLLOW_KEY{ "follow" };

namespace tesseract::common
{
namespace property_type
{
std::string createList(std::string_view type, std::size_t length)
{
  if (length == 0)
    return "List[" + std::string(type) + "]";

  return "List[" + std::string(type) + "," + std::to_string(length) + "]";
}

std::string createMap(std::string_view key, std::string_view type)
{
  return "Map[" + std::string(key) + "," + std::string(type) + "]";
}
std::string createMap(std::string_view type) { return createMap(STRING, type); }
}  // namespace property_type

namespace
{
bool schemaMatchesConfigShape(const PropertyTree& schema,
                              const YAML::Node& config,
                              std::set<std::string>& visited_types)
{
  if (!config || config.IsNull())
    return true;

  const auto type = schema.getAttribute(property_attribute::TYPE);
  if (!type.has_value())
    return schema.empty() || config.IsMap();

  const auto type_name = type->as<std::string>();
  if (type_name == property_type::ONEOF)
  {
    for (const auto& branch_name : schema.keys())
    {
      if (schemaMatchesConfigShape(schema.at(branch_name), config, visited_types))
        return true;
    }
    return false;
  }

  if (isSequenceType(type_name).has_value())
    return config.IsSequence();
  if (type_name == property_type::CONTAINER || isMapType(type_name).has_value())
    return config.IsMap();
  const auto accepts_derived = schema.getAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);
  if (accepts_derived.has_value() && accepts_derived->as<bool>())
    return config.IsMap();

  auto registry = SchemaRegistry::instance();
  if (registry->contains(type_name) && visited_types.insert(type_name).second)
  {
    const bool matches = schemaMatchesConfigShape(registry->get(type_name), config, visited_types);
    visited_types.erase(type_name);
    return matches;
  }

  return config.IsScalar();
}

bool schemaMatchesConfigShape(const PropertyTree& schema, const YAML::Node& config)
{
  std::set<std::string> visited_types;
  return schemaMatchesConfigShape(schema, config, visited_types);
}

std::string formatValidationErrors(const std::vector<std::string>& errors)
{
  std::string message = "PropertyTree configuration validation failed:";
  for (const auto& error : errors)
    message += "\n  - " + error;
  return message;
}

std::string childPath(const std::string& path, std::string_view child)
{
  if (path.empty())
    return std::string(child);
  return path + "." + std::string(child);
}

std::string errorPath(const std::string& path) { return path.empty() ? "(root)" : path; }

void prependErrorPath(std::vector<std::string>& errors, const std::string& path)
{
  for (auto& error : errors)
  {
    if (error.rfind("(root)", 0) == 0)
      error.replace(0, 6, path);
    else
    {
      std::string prefixed = path;
      prefixed += ": ";
      prefixed += error;
      error = std::move(prefixed);
    }
  }
}
}  // namespace

PropertyTreeValidationError::PropertyTreeValidationError(std::vector<std::string> errors)
  : std::runtime_error(formatValidationErrors(errors)), errors_(std::move(errors))
{
}

const std::vector<std::string>& PropertyTreeValidationError::errors() const noexcept { return errors_; }

PropertyTree::PropertyTree(const PropertyTree& other)
  : value_(YAML::Clone(other.value_))
  , children_(other.children_)
  , auto_validators_(other.auto_validators_)
  , validators_(other.validators_)
  , merged_config_presence_(other.merged_config_presence_)
{
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

  // Copy and clone attributes
  attributes_.clear();
  for (const auto& [k, node] : other.attributes_)
    attributes_[k] = YAML::Clone(node);

  // Copy children and validators
  children_ = other.children_;
  auto_validators_ = other.auto_validators_;
  validators_ = other.validators_;
  merged_config_presence_ = other.merged_config_presence_;

  return *this;
}

std::vector<std::string> PropertyTree::applyConfig(const YAML::Node& config, bool allow_extra_properties)
{
  std::vector<std::string> errors;
  applyConfigImpl(config, allow_extra_properties, "", errors);
  auto validation_errors = validate(allow_extra_properties);
  errors.insert(errors.end(),
                std::make_move_iterator(validation_errors.begin()),
                std::make_move_iterator(validation_errors.end()));
  return errors;
}

void PropertyTree::applyConfigImpl(const YAML::Node& config,
                                   bool allow_extra_properties,
                                   const std::string& path,
                                   std::vector<std::string>& errors)
{
  merged_config_presence_ = (config && !config.IsNull()) ? ConfigPresence::PRESENT : ConfigPresence::ABSENT;

  // Handle oneOf nodes up front
  auto t = getAttribute(property_attribute::TYPE);
  if (t.has_value() && t->as<std::string>() == property_type::ONEOF)
  {
    if (children_.empty())
      throw std::runtime_error("oneOf schema does not define any branches");

    // Required/default handling is applied after an absent value resolves to
    // the first branch.
    std::string chosen;
    if (!config || config.IsNull())
    {
      chosen = children_.front().first;
    }
    else
    {
      // Find exactly one branch matching the YAML node shape. Map branches retain
      // the existing required-key discriminator used by structural oneOf schemas.
      std::vector<std::string> candidates;
      for (const auto& [branch_name, branch_schema] : children_)
      {
        if (!schemaMatchesConfigShape(branch_schema, config))
          continue;

        bool matches = true;
        if (config.IsMap())
        {
          const auto accepts_derived = branch_schema.getAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);
          if (branch_schema.empty() && accepts_derived.has_value() && accepts_derived->as<bool>())
            matches = static_cast<bool>(config["class"]);

          for (const auto& key : branch_schema.keys())
          {
            if (branch_schema.at(key).isRequired() && !config[key])
            {
              matches = false;
              break;
            }
          }
        }

        if (matches)
          candidates.push_back(branch_name);
      }

      if (candidates.size() == 1)
      {
        chosen = candidates.front();
      }
      else if (candidates.size() > 1)
      {
        // Shape alone cannot distinguish alternatives such as two sequence
        // types. In that case, retain only branches whose complete schema
        // validates the provided value.
        std::vector<std::string> valid_candidates;
        for (const auto& candidate : candidates)
        {
          try
          {
            PropertyTree candidate_schema = at(candidate);
            if (candidate_schema.applyConfig(config, allow_extra_properties).empty())
              valid_candidates.push_back(candidate);
          }
          catch (const std::exception& exception)
          {
            // A branch that cannot merge the value is not a match.
            static_cast<void>(exception);
          }
        }

        if (valid_candidates.size() > 1)
        {
          errors.push_back(errorPath(path) + ": oneOf: multiple branches match");
          return;
        }
        if (valid_candidates.size() == 1)
          chosen = valid_candidates.front();
      }
    }

    if (chosen.empty())
    {
      errors.push_back(errorPath(path) + ": oneOf: no branch matches the provided value");
      return;
    }

    PropertyTree selected_schema = at(chosen);

    // Attributes and validators declared on the oneOf apply to every branch.
    // The selected branch's concrete type replaces the oneOf type.
    for (const auto& [name, attribute] : attributes_)
    {
      if (name != property_attribute::TYPE)
        selected_schema.setAttribute(name, YAML::Clone(attribute));
    }
    selected_schema.validators_.insert(selected_schema.validators_.end(), validators_.begin(), validators_.end());
    *this = std::move(selected_schema);

    applyConfigImpl(config, allow_extra_properties, path, errors);
    return;
  }

  // Leaf-schema override for both maps and sequences:
  // If this schema node has no children, but the user provided
  // either a map or a sequence, just store it wholesale.
  const auto configured_type = getAttribute(property_attribute::TYPE);
  const bool is_container =
      configured_type.has_value() && configured_type->as<std::string>() == property_type::CONTAINER;
  if (!is_container && children_.empty() && config && (config.IsMap() || config.IsSequence()))
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
    // Handle inline oneOf children first: run branch selection on parent's config,
    // hoist chosen branch's children into this node, then remove the oneOf child.
    std::set<std::string> parent_keys;
    for (const auto& [key, child_schema] : children_)
    {
      auto child_type = child_schema.getAttribute(property_attribute::TYPE);
      if (!child_type.has_value() || child_type->as<std::string>() != property_type::ONEOF)
        parent_keys.insert(key);
    }

    std::set<std::string> hoisted_keys;
    std::vector<std::pair<std::string, PropertyTree>> rebuilt_children;
    rebuilt_children.reserve(children_.size());
    for (auto& [key, child_schema] : children_)
    {
      auto child_type = child_schema.getAttribute(property_attribute::TYPE);
      if (child_type.has_value() && child_type->as<std::string>() == property_type::ONEOF)
      {
        // Run oneOf branch selection using parent's full config
        PropertyTree oneof_copy = child_schema;
        const std::size_t error_count = errors.size();
        oneof_copy.applyConfigImpl(config, true, path, errors);
        if (errors.size() != error_count)
          continue;

        // Hoist only the chosen branch's declared children into this node.
        // Parent-level shared fields appear as extras inside the temporary oneOf merge
        // and must not overwrite the parent's existing schema entries.
        for (auto& [hoisted_key, hoisted_child] : oneof_copy.children_)
        {
          if (hoisted_child.hasAttribute(EXTRA_KEY))
            continue;

          if (parent_keys.count(hoisted_key) > 0 || hoisted_keys.count(hoisted_key) > 0)
            throw std::runtime_error("inline oneOf: branch property '" + hoisted_key +
                                     "' conflicts with another property in the parent container");

          rebuilt_children.emplace_back(hoisted_key, std::move(hoisted_child));
          hoisted_keys.insert(hoisted_key);
        }
      }
      else
      {
        rebuilt_children.emplace_back(key, std::move(child_schema));
      }
    }
    children_ = std::move(rebuilt_children);

    // Fill declared children (skip already-merged hoisted keys)
    for (auto& [key, child_schema] : children_)
    {
      if (hoisted_keys.count(key) > 0)
        continue;
      auto sub = config[key];
      child_schema.applyConfigImpl(sub, allow_extra_properties, childPath(path, key), errors);
    }

    // Handle extras
    if (!allow_extra_properties)
    {
      for (auto it = config.begin(); it != config.end(); ++it)
      {
        std::string key;
        try
        {
          key = it->first.as<std::string>();
        }
        catch (const std::exception& e)
        {
          errors.push_back(errorPath(path) + ": map key must be a string: " + e.what());
          continue;
        }
        if (find(key) == nullptr)
        {
          auto& extra_node = (*this)[key];
          extra_node.setAttribute(EXTRA_KEY, YAML::Node(true));
          extra_node.applyConfigImpl(it->second, allow_extra_properties, childPath(path, key), errors);
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
      children_.back().second.applyConfigImpl(
          elt, allow_extra_properties, childPath(path, children_.back().first), errors);
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
  const auto type = getAttribute(property_attribute::TYPE);
  if (type.has_value() && type->as<std::string>() == property_type::ONEOF &&
      merged_config_presence_ != ConfigPresence::UNMERGED)
  {
    return;
  }

  // An optional container omitted from the merged config does not activate
  // required fields declared inside that container. An explicitly present
  // container, including an empty map, still validates all descendants.
  if (!path.empty() && merged_config_presence_ == ConfigPresence::ABSENT && !isRequired())
    return;

  // check if it is an extra property not found in schema
  if (!allow_extra_properties && hasAttribute(EXTRA_KEY))
  {
    std::string final_path = path.empty() ? "(root)" : path;
    std::string msg(final_path);
    msg += ": property does not exist in schema";
    errors.push_back(msg);
  }

  // recurse ALL children
  for (const auto& [key, child] : children_)
  {
    if (merged_config_presence_ == ConfigPresence::ABSENT &&
        child.merged_config_presence_ == ConfigPresence::UNMERGED && !child.isRequired())
      continue;

    std::string child_path = path;
    if (!path.empty())
      child_path += ".";
    child_path += key;
    child.collectErrors(errors, child_path, allow_extra_properties);
  }

  // Optional absent values have nothing to validate. Present containers
  // intentionally store null values, so their validators must still run.
  if (!isRequired() && isNull() && merged_config_presence_ != ConfigPresence::PRESENT)
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

bool PropertyTree::isContainer() const
{
  const auto type = getAttribute(property_attribute::TYPE);
  return !children_.empty() || (type.has_value() && type->as<std::string>() == property_type::CONTAINER);
}

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
  if (name == property_attribute::REQUIRED || name == property_attribute::ENUM || name == property_attribute::TYPE ||
      name == property_attribute::MINIMUM_LENGTH || name == property_attribute::MAXIMUM_LENGTH ||
      name == property_attribute::ACCEPTS_DERIVED_TYPES)
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
    {
      auto_validators_.emplace_back([length = is_sequence.value().second](const PropertyTree& node,
                                                                          const std::string& path,
                                                                          std::vector<std::string>& errors) {
        validateSequence(node, length, path, errors);
      });

      // Also add validateCustomType for sequences if the element type is a custom type that needs validation
      std::string element_type = is_sequence.value().first;
      bool has_derived_types = hasAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);
      auto registry = SchemaRegistry::instance();
      if (has_derived_types || registry->contains(element_type))
      {
        auto_validators_.emplace_back(validateCustomType);
      }
      else
      {
        auto_validators_.emplace_back(
            [element_type](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
              if (!node.getValue().IsSequence())
                return;

              std::size_t index{ 0 };
              for (const auto& value : node.getValue())
              {
                PropertyTree element_schema;
                element_schema.setAttribute(property_attribute::TYPE, element_type);
                auto element_errors = element_schema.applyConfig(value);
                prependErrorPath(element_errors, path + "[" + std::to_string(index++) + "]");
                errors.insert(errors.end(),
                              std::make_move_iterator(element_errors.begin()),
                              std::make_move_iterator(element_errors.end()));
              }
            });
      }
    }

    std::optional<std::pair<std::string, std::string>> is_map = isMapType(str_type);
    if (is_map.has_value())
    {
      auto_validators_.emplace_back(validateMap);

      // Also add validateCustomType for maps if the value type is a custom type that needs validation
      std::string map_value_type = is_map.value().second;
      bool has_derived_types = hasAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);
      auto registry = SchemaRegistry::instance();
      if (has_derived_types || registry->contains(map_value_type))
      {
        auto_validators_.emplace_back(validateCustomType);
      }
    }

    // Only validate as built-in or custom type if not a container type (List or Map)
    if (!is_sequence.has_value() && !is_map.has_value())
    {
      if (str_type == property_type::CONTAINER)
        auto_validators_.emplace_back(validateContainer);
      else if (str_type == property_type::STRING)
      {
        auto_validators_.emplace_back(validateTypeCast<std::string>);
        if (hasAttribute(property_attribute::MINIMUM_LENGTH) || hasAttribute(property_attribute::MAXIMUM_LENGTH))
          auto_validators_.emplace_back(validateStringLength);
      }
      else if (str_type == property_type::BOOL)
        auto_validators_.emplace_back(validateTypeCast<bool>);
      else if (str_type == property_type::CHAR)
        auto_validators_.emplace_back(validateTypeCast<char>);
      else if (str_type == property_type::FLOAT32)
        auto_validators_.emplace_back(validateTypeCastWithRange<float>);
      else if (str_type == property_type::FLOAT64)
        auto_validators_.emplace_back(validateTypeCastWithRange<double>);
      else if (str_type == property_type::INT32)
        auto_validators_.emplace_back(validateTypeCastWithRange<int32_t>);
      else if (str_type == property_type::UINT32)
        auto_validators_.emplace_back(validateTypeCastWithRange<uint32_t>);
      else if (str_type == property_type::INT64)
        auto_validators_.emplace_back(validateTypeCastWithRange<int64_t>);
      else if (str_type == property_type::UINT64)
        auto_validators_.emplace_back(validateTypeCastWithRange<uint64_t>);
      else if (str_type == property_type::EIGEN_ISOMETRY_3D)
        auto_validators_.emplace_back(validateTypeCast<Eigen::Isometry3d>);
      // else if (str_type == property_type::EIGEN_MATRIX_2D)
      //   auto_validators_.emplace_back(validateTypeCast<Eigen::MatrixXd>);
      else if (str_type == property_type::EIGEN_VECTOR_XD)
      {
        auto_validators_.emplace_back(
            [](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
              validateSequence(node, 0, path, errors);
            });
        auto_validators_.emplace_back(validateTypeCast<Eigen::VectorXd>);
      }
      // else if (str_type == property_type::EIGEN_MATRIX_2D)
      //   auto_validators_.emplace_back(validateTypeCast<Eigen::Matrix2d>);
      else if (str_type == property_type::EIGEN_VECTOR_2D)
      {
        auto_validators_.emplace_back(
            [](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
              validateSequence(node, 2, path, errors);
            });
        auto_validators_.emplace_back(validateTypeCast<Eigen::Vector2d>);
      }
      // else if (str_type == property_type::EIGEN_MATRIX_3D)
      //   auto_validators_.emplace_back(validateTypeCast<Eigen::Matrix3d>);
      else if (str_type == property_type::EIGEN_VECTOR_3D)
      {
        auto_validators_.emplace_back(
            [](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
              validateSequence(node, 3, path, errors);
            });
        auto_validators_.emplace_back(validateTypeCast<Eigen::Vector3d>);
      }
      else
      {
        // For custom types that match registry schemas or accept derived types,
        // automatically add validateCustomType to validate against registry schemas.
        // Only add if type is registered in the schema registry or has acceptsDerivedTypes.
        bool has_derived_types = hasAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);
        auto registry = SchemaRegistry::instance();
        if (has_derived_types || registry->contains(str_type))
        {
          auto_validators_.emplace_back(validateCustomType);
        }
      }
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

    // extract the value if it exists (for leaves with attributes)
    if (node[std::string(VALUE_KEY)])
      tree.value_ = node[std::string(VALUE_KEY)];

    // extract children
    for (const auto& it : node)
    {
      const auto key = it.first.as<std::string>();
      if (key == std::string(ATTRIBUTES_KEY) || key == std::string(VALUE_KEY))
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

PropertyTreeBuilder& PropertyTreeBuilder::int32(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::INT32);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::uint32(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::UINT32);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::int64(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::INT64);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::uint64(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::UINT64);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::float32(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::FLOAT32);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::float64(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::FLOAT64);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::eigenIsometry3d(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::EIGEN_ISOMETRY_3D);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::eigenVectorXd(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::EIGEN_VECTOR_XD);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::eigenVector2d(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::EIGEN_VECTOR_2D);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::eigenVector3d(std::string_view name)
{
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::EIGEN_VECTOR_3D);
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

PropertyTreeBuilder& PropertyTreeBuilder::minimumLength(std::size_t length)
{
  current().setAttribute(property_attribute::MINIMUM_LENGTH, YAML::Node(length));
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::maximumLength(std::size_t length)
{
  current().setAttribute(property_attribute::MAXIMUM_LENGTH, YAML::Node(length));
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

PropertyTreeBuilder& PropertyTreeBuilder::acceptsDerivedTypes()
{
  current().setAttribute(property_attribute::ACCEPTS_DERIVED_TYPES, true);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::oneOf()
{
  current().setAttribute(property_attribute::TYPE, property_type::ONEOF);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::beginOneOf()
{
  std::string name = "__oneOf_" + std::to_string(inline_oneof_counter_++) + "__";
  auto& child = current()[name];
  child.setAttribute(property_attribute::TYPE, property_type::ONEOF);
  stack_.push_back(&child);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::endOneOf() { return done(); }

PropertyTreeBuilder& PropertyTreeBuilder::pluginContainer(std::string_view name,
                                                          std::string_view factory_base_type,
                                                          std::string_view plugin_section)
{
  // clang-format off
  container(name);
    string("default").done();
    customType("plugins", property_type::createMap(factory_base_type))
        .required()
        .acceptsDerivedTypes()
    .done();
  done();
  // clang-format on
  current().at(name).setAttribute(property_attribute::PLUGIN_BASE_TYPE, factory_base_type);
  current().at(name).setAttribute(property_attribute::PLUGIN_SECTION, plugin_section);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::pluginContainerMap(std::string_view name,
                                                             std::string_view factory_base_type,
                                                             std::string_view plugin_section)
{
  // Register the inner PluginInfoContainer schema idempotently
  std::string registry_key = std::string(factory_base_type) + "::PluginInfoContainer";
  auto reg = SchemaRegistry::instance();
  if (!reg->contains(registry_key))
    reg->registerSchema(registry_key, makePluginInfoContainerSchema(factory_base_type));

  // Create a Map[string, <registry_key>] child with custom type validation
  customType(name, property_type::createMap(registry_key)).done();
  current().at(name).setAttribute(property_attribute::PLUGIN_BASE_TYPE, factory_base_type);
  current().at(name).setAttribute(property_attribute::PLUGIN_SECTION, plugin_section);
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::done()
{
  if (stack_.size() <= 1)
    throw std::runtime_error("PropertyTreeBuilder::done() called at root level");
  stack_.pop_back();
  return *this;
}

PropertyTreeBuilder& PropertyTreeBuilder::compose(const PropertyTree& source)
{
  for (const auto& key : source.keys())
    current()[key] = source.at(key);
  return *this;
}

PropertyTree PropertyTreeBuilder::build()
{
  if (stack_.size() != 1)
    throw std::runtime_error("PropertyTreeBuilder::build() called with unclosed scopes — missing done() calls");
  return std::move(root_);
}

PropertyTree makePluginInfoContainerSchema(std::string_view factory_base_type)
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("default").done()
      .customType("plugins", property_type::createMap(factory_base_type))
          .required()
          .acceptsDerivedTypes()
          .done()
      .build();
  // clang-format on
}

std::optional<std::pair<std::string, std::size_t>> isSequenceType(std::string_view type)
{
  static std::once_flag seq_flag;
  static std::unique_ptr<std::regex> re;
  std::call_once(seq_flag, []() { re = std::make_unique<std::regex>(R"(^List\[([^,\[\]]+)(?:,(\d+))?\]$)"); });

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
  std::call_once(map_flag, []() { re = std::make_unique<std::regex>(R"(^Map\[([^\[\]]+),([^\[\]]+)\]$)"); });

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
    {
      std::string msg(path);
      msg += ": required property missing or null";
      errors.push_back(msg);
    }
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
    {
      std::string msg(path);
      msg += ": value '";
      msg += val;
      msg += "' not in enum list";
      errors.push_back(msg);
    }
  }
}

void validateStringLength(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  if (!node.getValue().IsScalar())
    return;

  const std::size_t length = node.getValue().as<std::string>().size();
  const auto minimum = node.getAttribute(property_attribute::MINIMUM_LENGTH);
  if (minimum.has_value())
  {
    const auto minimum_length = minimum->as<std::size_t>();
    if (length < minimum_length)
    {
      errors.push_back(path + ": string length " + std::to_string(length) + " is less than minimum " +
                       std::to_string(minimum_length));
    }
  }

  const auto maximum = node.getAttribute(property_attribute::MAXIMUM_LENGTH);
  if (maximum.has_value())
  {
    const auto maximum_length = maximum->as<std::size_t>();
    if (length > maximum_length)
    {
      errors.push_back(path + ": string length " + std::to_string(length) + " is greater than maximum " +
                       std::to_string(maximum_length));
    }
  }
}

void validateMap(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  if (!node.getValue().IsMap())
  {
    std::string msg(path);
    msg += ": value is not of type YAML::NodeType::Map";
    errors.push_back(msg);
  }
}

void validateSequence(const PropertyTree& node,
                      std::size_t length,
                      const std::string& path,
                      std::vector<std::string>& errors)
{
  if (!node.getValue().IsSequence())
  {
    std::string msg(path);
    msg += ": value is not of type YAML::NodeType::Sequence";
    errors.push_back(msg);
    return;
  }

  if (length != 0 && node.getValue().size() != length)
  {
    std::string msg(path);
    msg += ": sequence length ";
    msg += std::to_string(node.getValue().size());
    msg += " does not match expected ";
    msg += std::to_string(length);
    errors.push_back(msg);
  }
}

void validateContainer(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  if (!node.isContainer())
  {
    std::string msg(path);
    msg += ": property is not a container";
    errors.push_back(msg);
  }

  if (!node.isNull())
  {
    std::string msg(path);
    msg += ": property is a container but value is not null";
    errors.push_back(msg);
  }
}

void validateCustomType(const PropertyTree& node, const std::string& path, std::vector<std::string>& errors)
{
  const auto type_attr = node.getAttribute(property_attribute::TYPE);
  if (!type_attr.has_value())
  {
    std::string msg(path);
    msg += ": custom type validator was added but type attribute does not exist";
    errors.push_back(msg);
    return;
  }

  const auto type_str = type_attr.value().as<std::string>();

  // Missing values are handled by validateRequired when applicable. Attempting
  // to merge a null value into a nested oneOf schema would otherwise throw
  // instead of returning the required-field diagnostic.
  if (node.isNull())
    return;

  std::optional<std::pair<std::string, std::size_t>> is_sequence = isSequenceType(type_str);
  std::optional<std::pair<std::string, std::string>> is_map = isMapType(type_str);

  auto registry = SchemaRegistry::instance();
  bool accepts_derived = node.hasAttribute(property_attribute::ACCEPTS_DERIVED_TYPES);

  if (!is_sequence.has_value() && !is_map.has_value())
  {
    // Non-sequence, non-map type (scalar or simple map)
    std::string actual_type = type_str;
    bool has_type_field = false;

    // Check for "type" field in the node to determine actual type
    if (node.getValue().IsMap() && node.getValue()[std::string("type")])
    {
      has_type_field = true;
      try
      {
        actual_type = node.getValue()[std::string("type")].as<std::string>();
        if (!registry->isDerivedFrom(type_str, actual_type))
        {
          std::stringstream ss;
          ss << path << ": type '" << actual_type << "' does not derive from '" << type_str << "'";
          errors.push_back(ss.str());
          return;
        }
      }
      catch (const std::exception& e)
      {
        std::stringstream ss;
        ss << path << ": type field exists but cannot be extracted as string: " << e.what();
        errors.push_back(ss.str());
        return;
      }
    }

    // If no "type" field and acceptsDerivedTypes is set, check for plugin info structure
    if (!has_type_field && accepts_derived)
    {
      validatePluginInfo(node, type_str, path, errors);
      return;
    }

    // For the "type" field approach (or when acceptsDerivedTypes is not set)
    if (!registry->contains(actual_type))
    {
      std::stringstream ss;
      ss << path << ": no schema registry entry found for key: " << actual_type;
      errors.push_back(ss.str());
      return;
    }

    PropertyTree schema = registry->get(actual_type);
    auto sub_errors = schema.applyConfig(node.getValue());
    prependErrorPath(sub_errors, path);
    errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
  }
  else if (is_sequence.has_value())
  {
    // Sequence type
    std::string base_element_type = is_sequence.value().first;
    bool base_in_registry = registry->contains(base_element_type);

    if (!base_in_registry && !accepts_derived)
    {
      std::stringstream ss;
      ss << path << ": no schema registry entry found for key: " << base_element_type;
      errors.push_back(ss.str());
      return;
    }

    const YAML::Node& sequence = node.getValue();
    std::size_t idx = 0;
    for (auto it = sequence.begin(); it != sequence.end(); ++it, ++idx)
    {
      std::string actual_element_type = base_element_type;
      bool has_type_field = false;

      // Check for "type" field in each element
      if (it->IsMap() && (*it)[std::string("type")])
      {
        has_type_field = true;
        try
        {
          actual_element_type = (*it)[std::string("type")].as<std::string>();
          if (!registry->isDerivedFrom(base_element_type, actual_element_type))
          {
            std::stringstream ss;
            ss << path << "[" << idx << "]: type '" << actual_element_type << "' does not derive from '"
               << base_element_type << "'";
            errors.push_back(ss.str());
            continue;
          }

          if (!registry->contains(actual_element_type))
          {
            std::stringstream ss;
            ss << path << "[" << idx << "]: no schema registry entry found for derived type: " << actual_element_type;
            errors.push_back(ss.str());
            continue;
          }
        }
        catch (const std::exception& e)
        {
          std::stringstream ss;
          ss << path << "[" << idx << "]: type field exists but cannot be extracted as string: " << e.what();
          errors.push_back(ss.str());
          continue;
        }
      }

      // If no "type" field and acceptsDerivedTypes is set, check for plugin info structure
      if (!has_type_field && accepts_derived)
      {
        PropertyTree elem_node;
        elem_node.setValue(*it);
        std::stringstream ss;
        ss << path << "[" << idx << "]";
        std::string elem_path = ss.str();
        validatePluginInfo(elem_node, base_element_type, elem_path, errors);
        continue;
      }

      // Get the appropriate schema (actual or base)
      PropertyTree schema = registry->get(actual_element_type);

      std::stringstream ss;
      ss << path << "[" << idx << "]";
      std::string elem_path = ss.str();
      PropertyTree copy_schema(schema);
      auto sub_errors = copy_schema.applyConfig(*it);
      prependErrorPath(sub_errors, elem_path);
      errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
    }
  }
  else if (is_map.has_value())
  {
    // Map type
    std::string map_value_type = is_map.value().second;
    bool base_in_registry = registry->contains(map_value_type);

    if (!base_in_registry && !accepts_derived)
    {
      std::stringstream ss;
      ss << path << ": no schema registry entry found for map value type: " << map_value_type;
      errors.push_back(ss.str());
      return;
    }

    const YAML::Node& map_node = node.getValue();
    if (!map_node.IsMap())
    {
      std::stringstream ss;
      std::string node_type = "unknown";
      if (map_node.IsNull())
        node_type = "null";
      else if (map_node.IsScalar())
        node_type = "scalar";
      else if (map_node.IsSequence())
        node_type = "sequence";

      ss << path << ": expected a map but got " << node_type;
      errors.push_back(ss.str());
      return;
    }

    std::size_t map_index = 0;
    for (auto it = map_node.begin(); it != map_node.end(); ++it, ++map_index)
    {
      if (!it->first.IsScalar())
      {
        std::stringstream ss;
        ss << path << ": map key at index " << map_index << " is not a string";
        errors.push_back(ss.str());
        continue;
      }

      auto key = it->first.as<std::string>();
      YAML::Node value_node = it->second;

      std::string actual_value_type = map_value_type;
      bool has_type_field = false;

      // Check for "type" field in each value
      if (value_node.IsMap() && value_node[std::string("type")])
      {
        has_type_field = true;
        try
        {
          actual_value_type = value_node[std::string("type")].as<std::string>();
          if (!registry->isDerivedFrom(map_value_type, actual_value_type))
          {
            std::stringstream ss;
            ss << path << "[" << key << "]: type '" << actual_value_type << "' does not derive from '" << map_value_type
               << "'";
            errors.push_back(ss.str());
            continue;
          }

          if (!registry->contains(actual_value_type))
          {
            std::stringstream ss;
            ss << path << "[" << key << "]: no schema registry entry found for derived type: " << actual_value_type;
            errors.push_back(ss.str());
            continue;
          }
        }
        catch (const std::exception& e)
        {
          std::stringstream ss;
          ss << path << "[" << key << "]: type field exists but cannot be extracted as string: " << e.what();
          errors.push_back(ss.str());
          continue;
        }
      }

      // If no "type" field and acceptsDerivedTypes is set, check for plugin info structure
      if (!has_type_field && accepts_derived)
      {
        PropertyTree elem_node;
        elem_node.setValue(value_node);
        std::stringstream ss;
        ss << path << "[" << key << "]";
        std::string elem_path = ss.str();
        validatePluginInfo(elem_node, map_value_type, elem_path, errors);
        continue;
      }

      // Get the appropriate schema (actual or base)
      PropertyTree schema = registry->get(actual_value_type);

      std::stringstream ss;
      ss << path << "[" << key << "]";
      std::string elem_path = ss.str();
      PropertyTree copy_schema(schema);
      auto sub_errors = copy_schema.applyConfig(value_node);
      prependErrorPath(sub_errors, elem_path);
      errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
    }
  }
}

void validatePluginInfo(const PropertyTree& node,
                        const std::string& base_type,
                        const std::string& path,
                        std::vector<std::string>& errors)
{
  // Validate that the node contains "class" and "config" fields (plugin info structure)
  const YAML::Node& value = node.getValue();

  if (!value.IsMap())
  {
    std::string node_type = "unknown";
    if (value.IsNull())
      node_type = "null";
    else if (value.IsScalar())
      node_type = "scalar";
    else if (value.IsSequence())
      node_type = "sequence";

    std::string msg(path);
    msg += ": expected a plugin info structure with 'class' and 'config' fields, but got ";
    msg += node_type;
    errors.push_back(msg);
    return;
  }

  // Check for required "class" field
  if (!value["class"])
  {
    std::string msg(path);
    msg += ": plugin info structure missing required 'class' field";
    errors.push_back(msg);
    return;
  }

  // Extract the derived type name
  std::string derived_type_name;
  try
  {
    derived_type_name = value["class"].as<std::string>();
  }
  catch (const std::exception& e)
  {
    std::string msg(path);
    msg += ".class: cannot extract class name: ";
    msg += e.what();
    errors.push_back(msg);
    return;
  }

  auto registry = SchemaRegistry::instance();

  // Check if the derived type is compatible with the base type
  if (!registry->isDerivedFrom(base_type, derived_type_name))
  {
    std::stringstream ss;
    ss << path << ".class: type '" << derived_type_name << "' does not derive from '" << base_type << "'";
    errors.push_back(ss.str());
    return;
  }

  // Check if schema exists for the derived type
  if (!registry->contains(derived_type_name))
  {
    std::stringstream ss;
    ss << path << ".class: no schema registry entry found for derived type: " << derived_type_name;
    errors.push_back(ss.str());
    return;
  }

  // Get the schema for the derived type
  PropertyTree schema = registry->get(derived_type_name);

  // Validate the config field against the schema
  if (value["config"])
  {
    PropertyTree config_copy = schema;
    auto sub_errors = config_copy.applyConfig(value["config"]);
    prependErrorPath(sub_errors, path + ".config");
    errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
  }
  else
  {
    // config field is optional but recommended - only warn if empty config is unusual
    PropertyTree config_copy = schema;
    YAML::Node empty_config;
    auto sub_errors = config_copy.applyConfig(empty_config);
    prependErrorPath(sub_errors, path + ".config");
    errors.insert(errors.end(), sub_errors.begin(), sub_errors.end());
  }
}

}  // namespace tesseract::common
