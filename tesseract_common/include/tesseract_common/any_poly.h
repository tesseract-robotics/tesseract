/**
 * @file any.h
 * @brief This a boost serializable any
 *
 * @author Levi Armstrong
 * @date February 27, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_ANY_POLY_H
#define TESSERACT_COMMON_ANY_POLY_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/stacktrace.hpp>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <memory>
#include <typeindex>
#include <unordered_map>
#include <tesseract_common/serialization.h>

namespace tesseract::common
{
template <typename T>
class AnyWrapper;
class AnyPoly;

template <class Archive, class T>
void serialize(Archive& ar, AnyWrapper<T>& obj);

template <class Archive>
void serialize(Archive& ar, AnyPoly& obj);

class AnyInterface
{
public:
  virtual ~AnyInterface();

  /**
   * @brief Get the type index of the object stored
   * @return The type index
   */
  virtual std::type_index getType() const;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  virtual std::unique_ptr<AnyInterface> clone() const = 0;

  // Operators
  bool operator==(const AnyInterface& rhs) const;
  bool operator!=(const AnyInterface& rhs) const;

protected:
  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  virtual bool equals(const AnyInterface& other) const = 0;
};

template <typename T>
class AnyWrapper : public AnyInterface
{
public:
  AnyWrapper() = default;
  explicit AnyWrapper(const T& value_) : value(value_) {}
  explicit AnyWrapper(T&& value_) : value(std::move(value_)) {}

  std::type_index getType() const override final { return typeid(T); }

  std::unique_ptr<AnyInterface> clone() const override final { return std::make_unique<AnyWrapper<T>>(value); }

  bool equals(const AnyInterface& other) const override final
  {
    const auto* derived_other = dynamic_cast<const AnyWrapper<T>*>(&other);
    if (derived_other == nullptr)
      return false;

    return (value == derived_other->value);
  }

  T value{};

private:
  template <class Archive, class U>
  friend void ::tesseract::common::serialize(Archive& ar, AnyWrapper<U>& obj);
};

class AnyPoly
{
public:
  AnyPoly() = default;   // Default constructor
  ~AnyPoly() = default;  // Default destructor
  AnyPoly(const AnyPoly& other);
  AnyPoly& operator=(const AnyPoly& other);
  AnyPoly(AnyPoly&& other) noexcept = default;
  AnyPoly& operator=(AnyPoly&& other) noexcept = default;
  AnyPoly(const AnyInterface& impl);

  /** Template Constructor for Moving Arbitrary Values */
  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T> && !std::is_same_v<std::decay_t<T>, AnyPoly>,
                                      bool> = true>
  AnyPoly(T&& value) : impl_(std::make_unique<AnyWrapper<std::decay_t<T>>>(std::forward<T>(value)))
  {
  }

  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T> && !std::is_same_v<std::decay_t<T>, AnyPoly>,
                                      bool> = true>
  AnyPoly& operator=(T&& value)
  {
    impl_ = std::make_unique<AnyWrapper<std::decay_t<T>>>(std::forward<T>(value));
    return *this;
  }

  /** Template Constructor for Copying Arbitrary Values */
  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T> && !std::is_same_v<std::decay_t<T>, AnyPoly>,
                                      bool> = true>
  AnyPoly(const T& value) : impl_(std::make_unique<AnyWrapper<T>>(value))
  {
  }

  /** Templated Copy Assignment Operator */
  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T> && !std::is_same_v<std::decay_t<T>, AnyPoly>,
                                      bool> = true>
  AnyPoly& operator=(const T& value)
  {
    impl_ = std::make_unique<AnyWrapper<T>>(value);
    return *this;
  }

  /**
   * @brief Get the stored derived type
   * @return The derived type index
   */
  std::type_index getType() const;

  /**
   * @brief Check if the poly type is null
   * @return True if null, otherwise false
   */
  bool isNull() const;

  /**
   * @brief Get the instruction being stored
   * @return The instruction
   * @throws If null
   */
  AnyInterface& get();
  const AnyInterface& get() const;

  // Type Casting
  template <typename T, typename std::enable_if_t<std::is_base_of_v<AnyInterface, T>, bool> = true>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("AnyPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T, typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T>, bool> = true>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("AnyPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return dynamic_cast<AnyWrapper<T>*>(impl_.get())->value;
  }

  template <typename T, typename std::enable_if_t<std::is_base_of_v<AnyInterface, T>, bool> = true>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("AnyPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  template <typename T, typename std::enable_if_t<!std::is_base_of_v<AnyInterface, T>, bool> = true>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("AnyPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return dynamic_cast<const AnyWrapper<T>*>(impl_.get())->value;
  }

  // Operators
  bool operator==(const AnyPoly& rhs) const;
  bool operator!=(const AnyPoly& rhs) const;

private:
  std::unique_ptr<AnyInterface> impl_;
  template <class Archive>
  friend void ::tesseract::common::serialize(Archive& ar, AnyPoly& obj);
};

using BoolAnyPoly = AnyWrapper<bool>;
using IntAnyPoly = AnyWrapper<int>;
using UnsignedAnyPoly = AnyWrapper<unsigned>;
using DoubleAnyPoly = AnyWrapper<double>;
using FloatAnyPoly = AnyWrapper<float>;
using StringAnyPoly = AnyWrapper<std::string>;
using SizeTAnyPoly = AnyWrapper<std::size_t>;

using VectorStringAnyPoly = AnyWrapper<std::vector<std::string>>;
using VectorBoolAnyPoly = AnyWrapper<std::vector<bool>>;
using VectorIntAnyPoly = AnyWrapper<std::vector<int>>;
using VectorUnsignedAnyPoly = AnyWrapper<std::vector<unsigned>>;
using VectorDoubleAnyPoly = AnyWrapper<std::vector<double>>;
using VectorFloatAnyPoly = AnyWrapper<std::vector<float>>;
using VectorSizeTAnyPoly = AnyWrapper<std::vector<std::size_t>>;

using UMapStringStringAnyPoly = AnyWrapper<std::unordered_map<std::string, std::string>>;
using UMapStringBoolAnyPoly = AnyWrapper<std::unordered_map<std::string, bool>>;
using UMapStringIntAnyPoly = AnyWrapper<std::unordered_map<std::string, int>>;
using UMapStringUnsignedAnyPoly = AnyWrapper<std::unordered_map<std::string, unsigned>>;
using UMapStringDoubleAnyPoly = AnyWrapper<std::unordered_map<std::string, double>>;
using UMapStringFloatAnyPoly = AnyWrapper<std::unordered_map<std::string, float>>;
using UMapStringSizeTAnyPoly = AnyWrapper<std::unordered_map<std::string, std::size_t>>;

}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_ANY_POLY_H
