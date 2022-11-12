/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef TESSERACT_COMMON_SHADER_PARAM_H
#define TESSERACT_COMMON_SHADER_PARAM_H

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

namespace tesseract_common
{
//
/// \brief forward declaration
class ShaderParamPrivate;

/// \brief a variant type that holds params that can be passed to a shader
class ShaderParam
{
public:
  using Ptr = std::shared_ptr<ShaderParam>;
  using ConstPtr = std::shared_ptr<const ShaderParam>;

  enum ParamType : uint16_t
  {
    /// \brief Type none
    PARAM_NONE = 0,

    /// \brief Float type parameter
    PARAM_FLOAT = 1,

    /// \brief Integer type parameter
    PARAM_INT = 2,

    /// \brief Float Buffer type parameter
    PARAM_FLOAT_BUFFER = 3,

    /// \brief Int Buffer type parameter
    PARAM_INT_BUFFER = 4,

    /// \brief texture type parameter
    PARAM_TEXTURE = 5,

    /// \brief cube map type parameter
    PARAM_TEXTURE_CUBE = 6,
  };

  /// \brief constructor
  ShaderParam();

  /// \brief copy constructor
  /// \param[in] _other Another ShaderParam
  ShaderParam(const ShaderParam& _other);

  /// \brief destructor
  ~ShaderParam();

  /// \brief Get the type of this parameter
  /// \return Type of this parameter
  ParamType Type() const;

  /// \brief Get the element count of this parameter's buffer
  /// \return Count of elements in this parameter's buffer
  uint32_t Count() const;

  /// \brief Set from another ShaderParam
  /// \param[in] _other Another ShaderParam
  /// \return Reference to this ShaderParam
  ShaderParam& operator=(const ShaderParam& _other);

  /// \brief Set this to be a float parameter
  /// \param[in] _value Value to set this parameter to
  void operator=(const float _value);

  /// \brief Set this to be an integer parameter
  /// \param[in] _value Value to set this parameter to
  void operator=(const int _value);

  /// \brief Set this to be a texture parameter
  /// \param[in] _value Value to set this parameter to
  /// \param[in] _type Type of texture
  /// \param[in] _uvSetIndex Texture coordinate set index
  void SetTexture(const std::string& _value,
                  ShaderParam::ParamType _type = ShaderParam::ParamType::PARAM_TEXTURE,
                  uint32_t _uvSetIndex = 0u);

  /// \brief Set this to be a buffer parameter
  /// \param[in] _count Number of 32-bit elements in the buffer
  void InitializeBuffer(uint32_t _count);

  /// \brief Copy a buffer to this parameter
  /// \param[in] _floatBuffer Source buffer to copy from
  void UpdateBuffer(float* _floatBuffer);

  /// \brief Copy a buffer to this parameter
  /// \param[in] _intBuffer Source buffer to copy from
  void UpdateBuffer(int* _intBuffer);

  /// \brief Get the value of this parameter if it is a float
  /// \param[out] _value variable the value will be copied to
  /// \return true if the parameter is the expected type
  bool Value(float* _value) const;

  /// \brief Get the value of this parameter if it is an int
  /// \param[out] _value variable the value will be copied to
  /// \return true if the parameter is the expected type
  bool Value(int* _value) const;

  /// \brief Get the value of this parameter if it is a texture
  /// \param[out] _value variable the value will be copied to
  /// \param[out] _uvSetIndex Texture coordinate set index
  /// \return true if the parameter filled
  bool Value(std::string& _value, uint32_t& _uvSetIndex) const;

  /// \brief Get the value of this parameter if it is a buffer
  /// \param[out] _buffer variable the value will be copied to
  /// \return true if the parameter is the expected type
  bool Buffer(std::shared_ptr<void>& _buffer) const;

  /// \brief private implementation
private:
  std::unique_ptr<ShaderParamPrivate> dataPtr;
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_SHADER_PARAM_H
