/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <tesseract_common/shader_type.h>
#include <console_bridge/console.h>

namespace tesseract_common
{
//////////////////////////////////////////////////
const char* ShaderUtil::names[static_cast<unsigned int>(ShaderType::ST_COUNT)] = {
  "UNKNOWN", "pixel", "vertex", "normal_map_object_space", "normal_map_tangent_space",
};

//////////////////////////////////////////////////
bool ShaderUtil::isValid(ShaderType type)
{
  return static_cast<unsigned int>(type) > 0 &&
         static_cast<unsigned int>(type) < static_cast<unsigned int>(ShaderType::ST_COUNT);
}

//////////////////////////////////////////////////
ShaderType ShaderUtil::sanitize(ShaderType type)
{
  // check if value within enum bounds
  if (!ShaderUtil::isValid(type))
  {
    CONSOLE_BRIDGE_logError("Invalid ShaderType value: %d", static_cast<unsigned int>(type));
    return ShaderType::ST_UNKNOWN;
  }

  return type;
}

//////////////////////////////////////////////////
std::string ShaderUtil::getName(ShaderType type)
{
  type = ShaderUtil::sanitize(type);
  return ShaderUtil::names[static_cast<unsigned int>(type)];
}

//////////////////////////////////////////////////
ShaderType ShaderUtil::getEnum(const std::string& name)
{
  // search over all enum elements
  for (unsigned int i = 0; i < static_cast<unsigned int>(ShaderType::ST_COUNT); ++i)
  {
    ShaderType format = static_cast<ShaderType>(i);

    // check if names match
    if (ShaderUtil::getName(format) == name)
    {
      return format;
    }
  }

  // no match found
  return ShaderType::ST_UNKNOWN;
}
}  // namespace tesseract_common
