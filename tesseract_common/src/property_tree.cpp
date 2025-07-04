#include <tesseract_common/property_tree.h>
#include <tesseract_common/schema_registry.h>
#include <tesseract_common/yaml_extenstions.h>
#include <Eigen/Geometry>

const static std::string ATTRIBUTES_KEY{ "_attributes" };
const static std::string VALUE_KEY{ "_value" };
const static std::string EXTRA_KEY{ "_extra" };
const static std::string ONEOF_KEY{ "_oneof" };
const static std::string FOLLOW_KEY{ "follow" };

namespace tesseract_common
{
namespace property_type
{
std::string createList(std::string_view type) { return std::string(type) + "[]"; };
std::string createMap(std::string_view type) { return std::string(type) + "{}"; };
}  // namespace property_type

PropertyTree::PropertyTree(const PropertyTree& other)
  : value_(YAML::Clone(other.value_))
  , follow_(YAML::Clone(other.follow_))
  , children_(other.children_)
  , keys_(other.keys_)
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

  // Copy children, keys and validators
  children_ = other.children_;
  keys_ = other.keys_;
  validators_ = other.validators_;

  // Copy oneOf
  if (other.oneof_ != nullptr)
    oneof_ = std::make_unique<PropertyTree>(*other.oneof_);

  return *this;
}

void PropertyTree::mergeConfig(const YAML::Node& config, bool allow_extra_properties)
{
  // Handle oneOf nodes up front
  auto t = getAttribute(property_attribute::TYPE);
  if (t.has_value() && t->as<std::string>() == property_type::ONEOF)
  {
    if (!config || !config.IsMap())
      std::throw_with_nested(std::runtime_error("oneOf schema expects a YAML map"));

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
          std::throw_with_nested(std::runtime_error("oneOf: multiple branches match"));

        chosen = branch_name;
      }
    }

    if (chosen.empty())
      std::throw_with_nested(std::runtime_error("oneOf: no branch matches the provided keys"));

    // Store schema
    oneof_ = std::make_unique<PropertyTree>(*this);

    // Flatten: replace this node's children with the chosen branch's children
    PropertyTree schema_copy = children_.at(chosen);
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
        if (children_.count(key) == 0)
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
    auto w = children_.find("*");
    if (w != children_.end())
      elem_schema = w->second;

    children_.clear();
    keys_.clear();
    int idx = 0;
    for (const auto& elt : config)
    {
      std::string key = std::to_string(idx++);
      keys_.push_back(key);
      children_[key] = elem_schema;
      children_[key].mergeConfig(elt, allow_extra_properties);
    }
  }
  // Otherwise leave value_ (possibly set by default) as-is.
}

void PropertyTree::validate(bool allow_extra_properties) const
{
  // check if it is an extra property not found in schema
  if (!allow_extra_properties && hasAttribute(EXTRA_KEY))
    std::throw_with_nested(std::runtime_error("Property does not exist in schema"));

  // recurse children
  for (const auto& kv : children_)
  {
    try
    {
      kv.second.validate();
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Validation failed for property: " + kv.first));
    }
  }

  // if not required and null skip validators
  if (!isRequired() && isNull())
    return;

  // run custom validators
  for (const auto& vfn : validators_)
    vfn(*this);
}

void PropertyTree::addValidator(ValidatorFn fn) { validators_.push_back(std::move(fn)); }

PropertyTree& PropertyTree::operator[](std::string_view key)
{
  auto k = std::string(key);
  auto it = children_.find(k);
  if (it == children_.end())
  {
    // first time insertion
    keys_.push_back(k);
    // default-construct in map
    it = children_.emplace(std::move(k), PropertyTree{}).first;
  }
  return it->second;
}

PropertyTree& PropertyTree::at(std::string_view key) { return children_.at(std::string(key)); }
const PropertyTree& PropertyTree::at(std::string_view key) const { return children_.at(std::string(key)); }

const PropertyTree* PropertyTree::find(std::string_view key) const
{
  auto it = children_.find(std::string(key));
  return it != children_.end() ? &it->second : nullptr;
}

void PropertyTree::setValue(const YAML::Node& v) { value_ = v; }
const YAML::Node& PropertyTree::getValue() const { return value_; }

bool PropertyTree::isNull() const { return value_.IsNull(); }

bool PropertyTree::isContainer() const { return !children_.empty(); }

std::vector<std::string> PropertyTree::keys() const { return keys_; }

void PropertyTree::setAttribute(std::string_view name, const YAML::Node& attr)
{
  attributes_[std::string(name)] = attr;

  if (name == property_attribute::REQUIRED)
    validators_.emplace_back(validateRequired);

  if (name == property_attribute::ENUM)
    validators_.emplace_back(validateEnum);

  if (name == property_attribute::TYPE)
  {
    const auto str_type = attr.as<std::string>();

    std::optional<std::string> is_sequence = isSequenceType(str_type);
    if (is_sequence.has_value())
      validators_.emplace_back(validateSequence);

    std::optional<std::string> is_map = isMapType(str_type);
    if (is_map.has_value())
      validators_.emplace_back(validateMap);

    if (str_type == property_type::CONTAINER)
      validators_.emplace_back(validateContainer);
    else if (str_type == property_type::STRING)
      validators_.emplace_back(validateTypeCast<std::string>);
    else if (str_type == property_type::BOOL)
      validators_.emplace_back(validateTypeCast<bool>);
    else if (str_type == property_type::CHAR)
      validators_.emplace_back(validateTypeCast<char>);
    else if (str_type == property_type::FLOAT)
      validators_.emplace_back(validateTypeCastWithRange<float>);
    else if (str_type == property_type::DOUBLE)
      validators_.emplace_back(validateTypeCastWithRange<double>);
    else if (str_type == property_type::INT)
      validators_.emplace_back(validateTypeCastWithRange<int>);
    else if (str_type == property_type::UNSIGNED_INT)
      validators_.emplace_back(validateTypeCastWithRange<unsigned int>);
    else if (str_type == property_type::LONG_INT)
      validators_.emplace_back(validateTypeCastWithRange<long int>);
    else if (str_type == property_type::LONG_UNSIGNED_INT)
      validators_.emplace_back(validateTypeCastWithRange<long unsigned int>);
    else if (str_type == property_type::EIGEN_ISOMETRY_3D)
      validators_.emplace_back(validateTypeCast<Eigen::Isometry3d>);
    // else if (str_type == property_type::EIGEN_MATRIX_2D)
    //   validators_.emplace_back(validateTypeCast<Eigen::MatrixXd>);
    else if (str_type == property_type::EIGEN_VECTOR_XD)
    {
      validators_.emplace_back(validateSequence);
      validators_.emplace_back(validateTypeCast<Eigen::VectorXd>);
    }
    // else if (str_type == property_type::EIGEN_MATRIX_2D)
    //   validators_.emplace_back(validateTypeCast<Eigen::Matrix2d>);
    else if (str_type == property_type::EIGEN_VECTOR_2D)
    {
      validators_.emplace_back(validateSequence);
      validators_.emplace_back(validateTypeCast<Eigen::Vector2d>);
    }
    // else if (str_type == property_type::EIGEN_MATRIX_3D)
    //   validators_.emplace_back(validateTypeCast<Eigen::Matrix3d>);
    else if (str_type == property_type::EIGEN_VECTOR_3D)
    {
      validators_.emplace_back(validateSequence);
      validators_.emplace_back(validateTypeCast<Eigen::Vector3d>);
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
  if (node.IsMap() && node[FOLLOW_KEY] && node[FOLLOW_KEY].IsScalar())
  {
    if (node.size() > 1)
      std::throw_with_nested(std::runtime_error("'follow' cannot be mixed with other entries"));

    try
    {
      auto key = node[FOLLOW_KEY].as<std::string>();
      auto registry = SchemaRegistry::instance();
      return registry->contains(key) ? registry->get(key) : SchemaRegistry::loadFile(key);
    }
    catch (const std::exception& e)
    {
      std::throw_with_nested(e);
    }
  }

  PropertyTree tree;
  tree.value_ = node;
  if (node.IsMap())
  {
    // extract attributes if it exists
    if (node[ATTRIBUTES_KEY] && node[ATTRIBUTES_KEY].IsMap())
    {
      for (const auto& it : node[ATTRIBUTES_KEY])
      {
        const auto key = it.first.as<std::string>();
        tree.attributes_[key] = it.second;
      }
    }

    // extract oneof if it exist
    if (node[ONEOF_KEY] && node[ONEOF_KEY].IsMap())
      tree.oneof_ = std::make_unique<PropertyTree>(fromYAML(node[ONEOF_KEY]));

    // extract children
    for (const auto& it : node)
    {
      const auto key = it.first.as<std::string>();
      if (key == ATTRIBUTES_KEY || key == ONEOF_KEY)
        continue;

      tree.children_[key] = fromYAML(it.second);
    }
  }
  else if (node.IsSequence())
  {
    int idx = 0;
    for (const auto& it : node)
      tree.children_[std::to_string(idx++)] = fromYAML(it);
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

    node[ATTRIBUTES_KEY] = attr_node;
  }

  if (keys_.size() != children_.size())
    std::throw_with_nested(std::runtime_error("PropertyTree, keys_ and children_ are not the same size"));

  // emit children
  for (const auto& key : keys_)
  {
    const PropertyTree& child = children_.at(key);
    // If the property is not required and is null then skip when excluding attributes
    if (exclude_attributes && !child.isRequired() && child.isNull())
      continue;

    node[key] = child.toYAML(exclude_attributes);
  }

  // emit oneof
  if (!exclude_attributes && oneof_ != nullptr)
    node[ONEOF_KEY] = oneof_->toYAML(exclude_attributes);

  // if leaf (no children) but value present, emit under 'value'
  if (children_.empty() && value_)
    node[VALUE_KEY] = value_;

  return node;
}

PropertyTree::operator bool() const noexcept { return (!children_.empty() || !value_.IsNull()); }

std::optional<std::string> isSequenceType(std::string_view type)
{
  if (type.size() < 2)
    return std::nullopt;

  if (type.substr(type.size() - 2) != "[]")
    return std::nullopt;

  return std::string(type.substr(0, type.size() - 2));
}

std::optional<std::string> isMapType(std::string_view type)
{
  if (type.size() < 2)
    return std::nullopt;

  if (type.substr(type.size() - 2) != "{}")
    return std::nullopt;

  return std::string(type.substr(0, type.size() - 2));
}

void validateRequired(const PropertyTree& node)
{
  auto req_attr = node.getAttribute(property_attribute::REQUIRED);
  if (req_attr && req_attr->as<bool>())
  {
    // if leaf node with no value or null
    if (!node.isContainer() && node.isNull())
      std::throw_with_nested(std::runtime_error("Required property missing or null"));
  }
}

void validateEnum(const PropertyTree& node)
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
    std::throw_with_nested(std::runtime_error("Property value '" + val + "' not in enum list"));
  }
}

void validateMap(const PropertyTree& node)
{
  if (!node.getValue().IsMap())
    std::throw_with_nested(std::runtime_error("Property value is not of type YAML::NodeType::Map"));
}

void validateSequence(const PropertyTree& node)
{
  if (!node.getValue().IsSequence())
    std::throw_with_nested(std::runtime_error("Property value is not of type YAML::NodeType::Sequence"));
}

void validateContainer(const PropertyTree& node)
{
  if (!node.isContainer())
    std::throw_with_nested(std::runtime_error("Property is not a container"));

  if (!node.isNull())
    std::throw_with_nested(std::runtime_error("Property is a container but value is not null"));
}

void validateCustomType(const PropertyTree& node)
{
  const auto type_attr = node.getAttribute(property_attribute::TYPE);
  if (!type_attr.has_value())
    std::throw_with_nested(std::runtime_error("Custom type validator was added buy type attribute does not exist"));

  const auto type_str = type_attr.value().as<std::string>();
  std::optional<std::string> is_sequence = isSequenceType(type_str);

  auto registry = SchemaRegistry::instance();
  if (!is_sequence.has_value())
  {
    if (!registry->contains(type_str))
      std::throw_with_nested(std::runtime_error("No scheme registry entry found for key: " + type_str));

    PropertyTree schema = registry->get(type_str);
    schema.mergeConfig(node.getValue());
    schema.validate();
  }
  else
  {
    if (!registry->contains(is_sequence.value()))
      std::throw_with_nested(std::runtime_error("No scheme registry entry found for key: " + is_sequence.value()));

    const YAML::Node& sequence = node.getValue();
    PropertyTree schema = registry->get(is_sequence.value());
    for (auto it = sequence.begin(); it != sequence.end(); ++it)
    {
      PropertyTree copy_schema(schema);
      copy_schema.mergeConfig(*it);
      copy_schema.validate();
    }
  }
}

}  // namespace tesseract_common
