#ifndef TESSERACT_COMMON_SCHEMA_REGISTRATION_H
#define TESSERACT_COMMON_SCHEMA_REGISTRATION_H

#include <string>
#include <functional>

namespace tesseract_common
{
class PropertyTree;

struct SchemaRegistrar
{
  SchemaRegistrar(const std::string& key, const std::string& path);

  SchemaRegistrar(const std::string& key, const std::function<PropertyTree()>& fn);
};

}  // namespace tesseract_common

/// Macro to register either a file‐based schema or a function‐built schema
#define TESSERACT_REGISTER_SCHEMA(KEY, SCHEMA_SOURCE)                                                                  \
  namespace                                                                                                            \
  {                                                                                                                    \
  /* now a const POD, linter is happy */                                                                               \
  static const int _reg_##KEY = []() -> int {                                                                          \
    using namespace tesseract_common;                                                                                  \
    /* calls the appropriate SchemaRegistrar constructor */                                                            \
    SchemaRegistrar(#KEY, SCHEMA_SOURCE);                                                                              \
    return 0;                                                                                                          \
  }();                                                                                                                 \
  }

#endif  // TESSERACT_COMMON_SCHEMA_REGISTRATION_H
