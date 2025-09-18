/**
 * @file any.h
 * @brief This a boost serializable any
 *
 * @author Levi Armstrong
 * @date February 27, 2021
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <boost/stacktrace.hpp>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <memory>
#include <typeindex>
#include <unordered_map>
#include <tesseract_common/serialization.h>

namespace tesseract_common
{
class AnyInterface
{
public:
  virtual ~AnyInterface() = default;

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

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
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
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(AnyInterface);
    ar& boost::serialization::make_nvp("value", value);
  }
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

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
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

}  // namespace tesseract_common

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_common::AnyInterface)
BOOST_CLASS_TRACKING(tesseract_common::AnyInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::AnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::AnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::BoolAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::BoolAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::IntAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::IntAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UnsignedAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UnsignedAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::DoubleAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::DoubleAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::FloatAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::FloatAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::StringAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::StringAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::SizeTAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::SizeTAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorStringAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorStringAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorBoolAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorBoolAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorIntAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorIntAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorUnsignedAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorUnsignedAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorDoubleAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorDoubleAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorFloatAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorFloatAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::VectorSizeTAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::VectorSizeTAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringStringAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringStringAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringBoolAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringBoolAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringIntAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringIntAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringUnsignedAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringUnsignedAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringDoubleAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringDoubleAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringFloatAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringFloatAnyPoly, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::UMapStringSizeTAnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::UMapStringSizeTAnyPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMON_ANY_POLY_H
