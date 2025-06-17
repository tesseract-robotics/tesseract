#include <tesseract_common/property_tree.h>
#include <tesseract_common/schema_registry.h>
#include <tesseract_common/yaml_extenstions.h>
#include <Eigen/Geometry>

const static std::string ATTRIBUTES_KEY{ "attributes" };
const static std::string VALUE_KEY{ "value" };
const static std::string FOLLOW_KEY{ "follow" };
const static std::string EXTRA_KEY{ "_extra" };

namespace tesseract_common
{
namespace property_type
{
std::string createList(std::string_view type) { return std::string(type) + "[]"; };
std::string createMap(std::string_view key_type, std::string_view value_type)
{
  return "{" + std::string(key_type) + " : " + std::string(value_type) + "}";
};
}  // namespace property_type

void PropertyTree::mergeConfig(const YAML::Node& config, bool allow_extra_properties)
{
  // 0) Leaf-schema override for both maps and sequences:
  //    If this schema node has no children, but the user provided
  //    either a map or a sequence, just store it wholesale.
  if (children_.empty() && config && (config.IsMap() || config.IsSequence()))
  {
    value_ = config;
    return;
  }

  // 1) Apply default if no config & not required
  auto def_it = attributes_.find(std::string(property_attribute::DEFAULT));
  bool required = isRequired();
  if ((!config || config.IsNull()) && def_it != attributes_.end() && !required)
    value_ = YAML::Clone(def_it->second);

  // 2) Scalar or (now only non‐leaf) sequence override
  if (config && config.IsScalar())
    value_ = config;

  // 3) Map: recurse into declared children
  if (config && config.IsMap())
  {
    // 3a) Fill declared children
    for (auto& [key, child_schema] : children_)
    {
      auto sub = config[key];
      child_schema.mergeConfig(sub, allow_extra_properties);
    }

    // 3b) Handle extras
    if (!allow_extra_properties)
    {
      for (auto it = config.begin(); it != config.end(); ++it)
      {
        const auto& key = it->first.as<std::string>();
        if (children_.count(key) == 0)
        {
          auto& extra_node = children_[key];
          extra_node.setAttribute(EXTRA_KEY, YAML::Node(true));
          extra_node.mergeConfig(it->second, allow_extra_properties);
        }
      }
    }
  }
  // 4) Sequence (non-leaf): apply wildcard or numeric schema
  else if (config && config.IsSequence())
  {
    PropertyTree elem_schema;
    auto w = children_.find("*");
    if (w != children_.end())
      elem_schema = w->second;

    children_.clear();
    int idx = 0;
    for (const auto& elt : config)
    {
      std::string key = std::to_string(idx++);
      children_[key] = elem_schema;
      children_[key].mergeConfig(elt, allow_extra_properties);
    }
  }
  // 5) Otherwise leave value_ (possibly set by default) as-is.
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

/// Add a custom validator for this node
void PropertyTree::addValidator(ValidatorFn fn) { validators_.push_back(std::move(fn)); }

PropertyTree& PropertyTree::get(std::string_view key) { return children_[std::string(key)]; }
const PropertyTree& PropertyTree::get(std::string_view key) const { return children_.at(std::string(key)); }

const PropertyTree* PropertyTree::find(std::string_view key) const
{
  auto it = children_.find(std::string(key));
  return it != children_.end() ? &it->second : nullptr;
}

void PropertyTree::setValue(const YAML::Node& v) { value_ = v; }
const YAML::Node& PropertyTree::getValue() const { return value_; }

bool PropertyTree::isNull() const { return value_.IsNull(); }

bool PropertyTree::isContainer() const { return !children_.empty(); }

std::vector<std::string> PropertyTree::keys() const
{
  std::vector<std::string> res;
  res.reserve(children_.size());
  for (const auto& pair : children_)
    res.push_back(pair.first);
  return res;
}

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

    if (str_type == property_type::STRING)
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
      validators_.emplace_back(validateTypeCast<Eigen::VectorXd>);
    // else if (str_type == property_type::EIGEN_MATRIX_2D)
    //   validators_.emplace_back(validateTypeCast<Eigen::Matrix2d>);
    else if (str_type == property_type::EIGEN_VECTOR_2D)
      validators_.emplace_back(validateTypeCast<Eigen::Vector2d>);
    // else if (str_type == property_type::EIGEN_MATRIX_3D)
    //   validators_.emplace_back(validateTypeCast<Eigen::Matrix3d>);
    else if (str_type == property_type::EIGEN_VECTOR_3D)
      validators_.emplace_back(validateTypeCast<Eigen::Vector3d>);
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
      throw std::runtime_error("'follow' cannot be mixed with other entries");

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
    // extract attributes map
    if (node[ATTRIBUTES_KEY] && node[ATTRIBUTES_KEY].IsMap())
    {
      for (const auto& it : node[ATTRIBUTES_KEY])
      {
        const auto key = it.first.as<std::string>();
        tree.attributes_[key] = it.second;
      }
    }
    // extract children
    for (const auto& it : node)
    {
      const auto key = it.first.as<std::string>();
      if (key == ATTRIBUTES_KEY)
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
  // emit children
  for (const auto& pair : children_)
  {
    // If the property is not required and is null then skip when excluding attributes
    if (exclude_attributes && !pair.second.isRequired() && pair.second.isNull())
      continue;

    node[pair.first] = pair.second.toYAML(exclude_attributes);
  }

  // if leaf (no children) but value present, emit under 'value'
  if (children_.empty() && value_)
    node[VALUE_KEY] = value_;

  return node;
}

std::optional<std::string> isSequenceType(std::string_view type)
{
  if (type.size() < 2)
    return std::nullopt;

  if (type.substr(type.size() - 2) != "[]")
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
  if (node.getValue().IsNull() && !node.getValue().IsMap())
    std::throw_with_nested(std::runtime_error("Property value is not of type YAML::NodeType::Map"));
}

void validateSequence(const PropertyTree& node)
{
  if (!node.getValue().IsSequence())
    std::throw_with_nested(std::runtime_error("Property value is not of type YAML::NodeType::Sequence"));
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
