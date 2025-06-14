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

class PropertyTree
{
public:
  using ValidatorFn = std::function<bool(const PropertyTree&, std::vector<std::string>&, const std::string&)>;

  PropertyTree() = default;

  void mergeSchema(const PropertyTree& schema);

  bool validate(std::vector<std::string>& errors, const std::string& path = "") const;

  void addValidator(ValidatorFn fn);

  PropertyTree& get(std::string_view key);
  const PropertyTree& get(std::string_view key) const;
  const PropertyTree* find(std::string_view key) const;

  void setValue(const YAML::Node& v);
  const YAML::Node& getValue() const;

  std::vector<std::string> keys() const;

  void setAttribute(std::string_view name, const YAML::Node& attr);
  void setAttribute(std::string_view name, std::string_view attr);
  void setAttribute(std::string_view name, const char* attr);
  void setAttribute(std::string_view name, bool attr);
  void setAttribute(std::string_view name, int attr);
  void setAttribute(std::string_view name, double attr);

  std::optional<YAML::Node> getAttribute(std::string_view name) const;
  bool hasAttribute(std::string_view name) const;

  static PropertyTree fromYAML(const YAML::Node& node);

  YAML::Node toYAML() const;

private:
  YAML::Node value_;
  std::map<std::string, YAML::Node> attributes_;
  std::map<std::string, PropertyTree> children_;
  std::vector<ValidatorFn> validators_;
};

bool validateRequired(const PropertyTree& node, std::vector<std::string>& errors, const std::string& path);

bool validateRange(const PropertyTree& node, std::vector<std::string>& errors, const std::string& path);

bool validateEnum(const PropertyTree& node, std::vector<std::string>& errors, const std::string& path);

}  // namespace tesseract_common

#endif  // PROPERTY_TREE_H
