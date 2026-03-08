/**
 * @file schema_registration.cpp
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

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/schema_registry.h>
#include <tesseract/common/property_tree.h>

namespace tesseract::common
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
}  // namespace tesseract::common
