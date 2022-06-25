/**
 * @file type_erasure.h
 * @brief Boilerplate code for creating type erasures
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_TYPE_ERASURE_H
#define TESSERACT_COMMON_TYPE_ERASURE_H

#include <memory>
#include <typeindex>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <tesseract_common/serialization.h>

namespace tesseract_common
{
/** @brief This is the interface that all type erasures interfaces must inherit from */
struct TypeErasureInterface
{
  virtual ~TypeErasureInterface() = default;

  // This is not required for user defined implementation
  virtual bool equals(const TypeErasureInterface& other) const = 0;

  // This is not required for user defined implementation
  virtual std::type_index getType() const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual const void* recover() const = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<TypeErasureInterface> clone() const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/){};  // NOLINT
};

template <typename ConcreteType, typename Interface>
struct TypeErasureInstance : Interface
{
  using ValueType = ConcreteType;
  using InterfaceType = Interface;

  TypeErasureInstance() = default;

  explicit TypeErasureInstance(ConcreteType value) : value_(std::move(value)) {}

  explicit TypeErasureInstance(ConcreteType&& value) : value_(std::move(value)) {}

  const ValueType& get() const { return value_; }

  ValueType& get() { return value_; }

  void* recover() final { return &value_; }

  const void* recover() const final { return &value_; }

  std::type_index getType() const final { return std::type_index(typeid(ValueType)); }

  bool equals(const TypeErasureInterface& other) const final
  {
    return this->getType() == other.getType() && this->get() == *static_cast<const ValueType*>(other.recover());
  }

  ConcreteType value_;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    // If this line is removed a exception is thrown for unregistered cast need to too look into this.
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<Interface>(*this));
    ar& boost::serialization::make_nvp("impl", value_);
  }
};

template <typename F>
struct TypeErasureInstanceWrapper : F  // NOLINT
{
  using ValueType = typename F::ValueType;
  using InterfaceType = typename F::InterfaceType;

  TypeErasureInstanceWrapper() = default;
  TypeErasureInstanceWrapper(const ValueType& x) : F(x) {}
  TypeErasureInstanceWrapper(TypeErasureInstanceWrapper&& x) noexcept : F(std::move(x)) {}

  std::unique_ptr<TypeErasureInterface> clone() const final
  {
    return std::make_unique<TypeErasureInstanceWrapper<F>>(this->get());
  }

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    // If this line is removed a exception is thrown for unregistered cast need to too look into this.
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<F>(*this));
  }
};

template <typename Interface, template <typename> class Instance>
struct TypeErasureBase
{
private:
  template <typename T>
  using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_base_of<TypeErasureBase, uncvref_t<T>>::value, int>;

public:
  using InterfaceType = Interface;

  template <typename T, generic_ctor_enabler<T> = 0>
  TypeErasureBase(T&& value)  // NOLINT
    : value_(std::make_unique<TypeErasureInstanceWrapper<Instance<uncvref_t<T>>>>(value))
  {
  }

  TypeErasureBase() : value_(nullptr){};  // NOLINT

  // Destructor
  ~TypeErasureBase() = default;

  // Copy constructor
  TypeErasureBase(const TypeErasureBase& other) { value_ = other.value_->clone(); }

  // Move ctor.
  TypeErasureBase(TypeErasureBase&& other) noexcept { value_.swap(other.value_); }

  // Move assignment.
  TypeErasureBase& operator=(TypeErasureBase&& other) noexcept
  {
    value_.swap(other.value_);
    return (*this);
  }

  // Copy assignment.
  TypeErasureBase& operator=(const TypeErasureBase& other)
  {
    (*this) = TypeErasureBase(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  TypeErasureBase& operator=(T&& other)
  {
    (*this) = TypeErasureBase(std::forward<T>(other));
    return (*this);
  }

  std::type_index getType() const
  {
    if (value_ == nullptr)
      return std::type_index(typeid(nullptr));

    return value_->getType();
  }

  bool operator==(const TypeErasureBase& rhs) const { return value_->equals(*rhs.value_); }

  bool operator!=(const TypeErasureBase& rhs) const { return !operator==(rhs); }

  InterfaceType& interface() { return *static_cast<InterfaceType*>(value_.get()); }

  const InterfaceType& interface() const { return *static_cast<const InterfaceType*>(value_.get()); }

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("TypeErasureBase, tried to cast '" + std::string(getType().name()) + "' to '" +
                               std::string(typeid(T).name()) + "'!");

    auto* p = static_cast<uncvref_t<T>*>(value_->recover());
    return *p;
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("TypeErasureBase, tried to cast '" + std::string(getType().name()) + "' to '" +
                               std::string(typeid(T).name()) + "'!");

    const auto* p = static_cast<const uncvref_t<T>*>(value_->recover());
    return *p;
  }

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("value", value_);
  }

  std::unique_ptr<TypeErasureInterface> value_;
};

}  // namespace tesseract_common

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_common::TypeErasureInterface)
#endif  // TESSERACT_COMMON_TYPE_ERASURE_H
