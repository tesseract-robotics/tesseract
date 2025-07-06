#include <tesseract_common/schema_registration.h>
#include <tesseract_common/schema_registry.h>
#include <tesseract_common/property_tree.h>

namespace tesseract_common
{
SchemaRegistrar::SchemaRegistrar(const std::string& key, const std::string& path)
{
  auto reg = SchemaRegistry::instance();
  reg->registerSchemaFromFile(key, path);
}

SchemaRegistrar::SchemaRegistrar(const std::string& key, const std::function<PropertyTree()>& fn)
{
  auto reg = SchemaRegistry::instance();
  PropertyTree tree = fn();
  reg->registerSchema(key, tree);
}
}  // namespace tesseract_common
