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
#ifndef TESSERACT_COMMON_SHADER_PARAMS_H
#define TESSERACT_COMMON_SHADER_PARAMS_H

#include <memory>
#include <string>
#include <utility>

#include <tesseract_common/shader_param.h>

namespace tesseract_common
{
//
/// \brief forward declaration
class ShaderParamsPrivate;

/// \brief a map that holds params to be passed to a shader
class ShaderParams
{
public:
  using Ptr = std::shared_ptr<ShaderParams>;
  using ConstPtr = std::shared_ptr<const ShaderParams>;

  /// \brief forward declaration
  class IteratorPrivate;

  /// \brief Iterator for looping through params
  /// \remarks implements a forward-iterator
  class Iterator
  {
    /// \brief Default constructor
  public:
    Iterator();

    /// \brief Special constructor used by ShaderParams implementation
    /// \param[in] _dataPtr Pointer to private data.
    explicit Iterator(std::unique_ptr<IteratorPrivate> _dataPtr);

    /// \brief Copy constructor
    /// \param[in] _iter Another iterator
    Iterator(const Iterator& _iter);

    /// \brief Copy assignment
    /// \param[in] _iter Another iterator
    /// \return this
    Iterator& operator=(const Iterator& _iter);

    /// \brief Destructor
    ~Iterator();

    /// \brief Equality operator
    /// \param[in] _iter Another iterator
    /// \return True if input interator equal to this one, false otherwise.
    bool operator==(const Iterator& _iter);

    /// \brief Inequality operator
    /// \param[in] _iter Another iterator
    /// \return True if input interator is equal to this one,
    /// false otherwise
    bool operator!=(const Iterator& _iter);

    /// \brief Dereference operator
    const std::pair<const std::string, ShaderParam>& operator*();

    /// \brief Arrow dereference operator
    const std::pair<const std::string, ShaderParam>* operator->();

    /// \brief prefix increment
    Iterator& operator++();

    /// \brief postfix increment
    Iterator operator++(int);

  private:
    std::unique_ptr<IteratorPrivate> dataPtr;
  };

  /// \brief constructor
  ShaderParams();

  /// \brief destructor
  ~ShaderParams();

  /// \brief Access a param with a given name
  /// \param[in] _name Identifier for the parameter
  /// \returns parameter reference
  ShaderParam& operator[](const std::string& _name);

  /// \brief Access a param with a given name
  /// \param[in] _name Identifier for the parameter
  /// \returns const parameter reference
  const ShaderParam& operator[](const std::string& _name) const;

  /// \brief Iterator to first parameter
  /// \remarks Necessary for range-base for loop support
  /// \return Iterator pointing to first parameter.
  Iterator begin() const;

  /// \brief Iterator to one past last param
  /// \remarks Necessary for range-base for loop support
  /// \return Iterator pointing to one past last parameter.
  Iterator end() const;

  /// \brief Have the params changed?
  /// \internal
  /// \returns true if the parameters have changed
  bool IsDirty() const;

  /// \brief Resets the dirty flag
  /// \internal
  void ClearDirty();

  /// \brief private implementation
private:
  std::unique_ptr<ShaderParamsPrivate> dataPtr;
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_SHADER_PARAMS_H
