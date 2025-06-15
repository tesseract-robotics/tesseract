#include <tesseract_common/property_tree.h>

const static std::string ATTRIBUTES_KEY{ "attributes" };
const static std::string VALUE_KEY{ "value" };

namespace tesseract_common
{
void PropertyTree::mergeSchema(const PropertyTree& schema)
{
  // merge schema attributes
  for (const auto& pair : schema.attributes_)
  {
    if (!hasAttribute(pair.first))
      attributes_[pair.first] = pair.second;
  }

  // apply default value only when property is not required and no explicit value present
  auto default_it = attributes_.find(std::string(property_attribute::DEFAULT));
  if (default_it != attributes_.end())
  {
    // determine if property is marked required
    auto it = attributes_.find(std::string(property_attribute::REQUIRED));
    const bool required = (it != attributes_.end() && it->second.as<bool>());
    if (!required)
    {
      if (!value_ || value_.IsNull())
        value_ = YAML::Clone(default_it->second);
    }
  }

  // merge schema validators
  for (const auto& fn : schema.validators_)
    validators_.push_back(fn);

  // merge children recursively
  for (const auto& pair : schema.children_)
  {
    if (children_.find(pair.first) == children_.end())
      children_[pair.first] = PropertyTree();

    children_[pair.first].mergeSchema(pair.second);
  }
}

void PropertyTree::validate() const
{
  // run custom validators
  for (const auto& vfn : validators_)
    vfn(*this);

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

std::optional<YAML::Node> PropertyTree::getAttribute(std::string_view name) const
{
  auto it = attributes_.find(std::string(name));
  if (it != attributes_.end())
    return it->second;
  return std::nullopt;
}

bool PropertyTree::hasAttribute(std::string_view name) const
{
  auto it = attributes_.find(std::string(name));
  return (it != attributes_.end() && it->second && !it->second.IsNull());
}

PropertyTree PropertyTree::fromYAML(const YAML::Node& node)
{
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

YAML::Node PropertyTree::toYAML() const
{
  // Always emit a mapping if attributes exist or children exist
  if (!attributes_.empty() || !children_.empty())
  {
    YAML::Node node(YAML::NodeType::Map);
    // emit attributes first
    if (!attributes_.empty())
    {
      YAML::Node attr_node(YAML::NodeType::Map);
      for (const auto& pair : attributes_)
        attr_node[pair.first] = pair.second;

      node[ATTRIBUTES_KEY] = attr_node;
    }
    // emit children
    for (const auto& pair : children_)
      node[pair.first] = pair.second.toYAML();

    // if leaf (no children) but value present, emit under 'value'
    if (children_.empty() && value_)
      node[VALUE_KEY] = value_;

    return node;
  }

  // pure leaf without attributes: emit scalar/sequence directly
  return value_;
}

void validateRequired(const PropertyTree& node)
{
  auto req_attr = node.getAttribute(property_attribute::REQUIRED);
  if (req_attr && req_attr->as<bool>())
  {
    // if leaf node with no value or null
    if (!node.getValue() || node.getValue().IsNull())
      std::throw_with_nested(std::runtime_error("Required property missing or null"));
  }
}

void validateRange(const PropertyTree& node)
{
  auto min_attr = node.getAttribute(property_attribute::MINIMUM);
  auto max_attr = node.getAttribute(property_attribute::MAXIMUM);
  if (min_attr && max_attr)
  {
    const auto minv = min_attr->as<double>();
    const auto maxv = max_attr->as<double>();
    const auto val = node.getValue().as<double>();
    if (val < minv || val > maxv)
    {
      std::throw_with_nested(std::runtime_error("Property value " + std::to_string(val) + " out of range [" +
                                                std::to_string(minv) + "," + std::to_string(maxv) + "]"));
    }
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

}  // namespace tesseract_common
