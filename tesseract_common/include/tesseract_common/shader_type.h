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
#ifndef TESSERACT_COMMON_SHADER_TYPE_H
#define TESSERACT_COMMON_SHADER_TYPE_H

#include <string>

namespace tesseract_common
{
/// \enum ShaderType ShaderType.hh gz/rendering/ShaderType.hh
/// \brief Available types of shaders. Note that not all rendering-engines
/// will be able to use each type. They will instead default to the closest
/// alternative.
enum class ShaderType
{
  /// \brief Unknown or errant type
  ST_UNKNOWN = 0,

  /// \brief Per pixel lighting shader
  ST_PIXEL = 1,

  /// \brief Per vertex lighting shader
  ST_VERTEX = 2,

  /// \brief Object-space normal map shader
  ST_NORM_OBJ = 3,

  /// \brief Tangent-space normal map shader
  ST_NORM_TAN = 4,

  /// \brief Total number of shader types
  ST_COUNT = 5,
};

/// \class ShaderUtil ShaderType.hh gz/rendering/ShaderType.hh
/// \brief Provides supporting functions for ShaderType enum
class ShaderUtil
{
public:
  /// \brief Determine if given type is valid ShaderType enum
  /// \param[in] type Enum value to be evaluated
  /// \return True if the given type is valid
  static bool isValid(ShaderType type);

  /// \brief Sanitize given type. If the given value is invalid,
  /// ST_UNKNOWN will be returned, otherwise input will be returned
  /// unchanged.
  /// \param[in] type Shader type to be sanitized
  /// \return The santized shader type
  static ShaderType sanitize(ShaderType type);

  /// \brief Get human-readable name for shader type value.
  /// \param[in] type Shader type enum value
  /// \return The type name
  static std::string getName(ShaderType type);

  /// \brief Get enum value by human-readable name. The given string should
  /// match watch is returned by GetName. If an invalid name is given,
  /// ST_UNKNOWN will be returned.
  /// \param[in] name Name of the shader type to be retrieved
  /// \return The specified ShaderType enum value
  static ShaderType getEnum(const std::string& name);

  /// \brief Array of human-readable names for each ShaderType
private:
  static const char* names[static_cast<unsigned int>(ShaderType::ST_COUNT)];
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_SHADER_TYPE_H
