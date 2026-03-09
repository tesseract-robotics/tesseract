/**
 * @file schema_registration.h
 * @brief This is a schema registration class
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
#ifndef TESSERACT_COMMON_SCHEMA_REGISTRATION_H
#define TESSERACT_COMMON_SCHEMA_REGISTRATION_H

#include <string>
#include <functional>

namespace tesseract::common
{
class PropertyTree;

/**
 * @brief Register a schema from a file path.
 * @param key  Unique identifier for this schema.
 * @param path Path to a .yaml schema file.
 */
void registerSchema(const std::string& key, const std::string& path);

/**
 * @brief Register a schema from a function.
 * @param key Unique identifier for this schema.
 * @param fn  Function that returns a PropertyTree schema.
 */
void registerSchema(const std::string& key, const std::function<PropertyTree()>& fn);

/**
 * @brief Register that a derived type schema can be used where a base type is expected.
 * @param base_type    The base type name
 * @param derived_type The derived type name
 */
void registerSchemaDerivedType(const std::string& base_type, const std::string& derived_type);

}  // namespace tesseract::common

// first level: does the actual pasting
#define TESSERACT_SCHEMA_REG_PASTE(a, b) a##b

// second level: expands its arguments before passing to the first
#define TESSERACT_SCHEMA_REG_MAKE_NAME(a, b) TESSERACT_SCHEMA_REG_PASTE(a, b)

/// Macro to register either a file‐based schema or a function‐built schema
#define TESSERACT_SCHEMA_REGISTER(KEY, SCHEMA_SOURCE)                                                                  \
  namespace                                                                                                            \
  {                                                                                                                    \
  static const int TESSERACT_SCHEMA_REG_MAKE_NAME(schema_reg_, __COUNTER__) = []() -> int {                            \
    using namespace tesseract::common;                                                                                 \
    registerSchema(#KEY, SCHEMA_SOURCE);                                                                               \
    return 0;                                                                                                          \
  }();                                                                                                                 \
  }

/**
 * @brief Macro to register that a derived type can be used where a base type is expected.
 * @param BASE_TYPE    The base type name
 * @param DERIVED_TYPE The derived type name
 *
 * Example usage:
 * @code
 * TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::planning::BaseConstraint,
 *                                         tesseract::planning::JointPositionConstraint)
 * @endcode
 */
#define TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(BASE_TYPE, DERIVED_TYPE)                                                \
  namespace                                                                                                            \
  {                                                                                                                    \
  static const int TESSERACT_SCHEMA_REG_MAKE_NAME(derived_type_reg_, __COUNTER__) = []() -> int {                      \
    using namespace tesseract::common;                                                                                 \
    registerSchemaDerivedType(#BASE_TYPE, #DERIVED_TYPE);                                                              \
    return 0;                                                                                                          \
  }();                                                                                                                 \
  }

#endif  // TESSERACT_COMMON_SCHEMA_REGISTRATION_H
