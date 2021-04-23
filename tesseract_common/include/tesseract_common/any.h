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
#ifndef TESSERACT_COMMON_ANY_H
#define TESSERACT_COMMON_ANY_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <typeindex>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/export.hpp>
#include <boost/type_traits/is_virtual_base_of.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_ANY_EXPORT_KEY(any_t)                                                                                \
  BOOST_CLASS_EXPORT_KEY2(tesseract_common::detail_any::AnyInner<any_t>, #any_t)                                       \
  BOOST_CLASS_TRACKING(tesseract_common::detail_any::AnyInner<any_t>, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_ANY_EXPORT_IMPLEMENT(any_t)                                                                          \
  BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::detail_any::AnyInner<any_t>)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_ANY_EXPORT(any_t)                                                                                    \
  TESSERACT_ANY_EXPORT_KEY(any_t)                                                                                      \
  TESSERACT_ANY_EXPORT_IMPLEMENT(any_t)

namespace tesseract_common
{
#ifndef SWIG
namespace detail_any
{
struct AnyInnerBase
{
  AnyInnerBase() = default;
  virtual ~AnyInnerBase() = default;
  AnyInnerBase(const AnyInnerBase&) = delete;
  AnyInnerBase& operator=(const AnyInnerBase&) = delete;
  AnyInnerBase(AnyInnerBase&&) = delete;
  AnyInnerBase& operator=(AnyInnerBase&&) = delete;

  virtual bool operator==(const AnyInnerBase& rhs) const = 0;

  // This is not required for user defined implementation
  virtual bool operator!=(const AnyInnerBase& rhs) const = 0;

  // This is not required for user defined implementation
  virtual std::type_index getType() const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual const void* recover() const = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<AnyInnerBase> clone() const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/)
  {
  }
};

template <typename T>
struct AnyInner final : AnyInnerBase
{
  AnyInner() = default;
  ~AnyInner() override = default;
  AnyInner(const AnyInner&) = delete;
  AnyInner(AnyInner&&) = delete;
  AnyInner& operator=(const AnyInner&) = delete;
  AnyInner& operator=(AnyInner&&) = delete;

  // Constructors from T (copy and move variants).
  explicit AnyInner(T any_type) : any_type_(std::move(any_type)) {}
  explicit AnyInner(T&& any_type) : any_type_(std::move(any_type)) {}

  std::unique_ptr<AnyInnerBase> clone() const final { return std::make_unique<AnyInner>(any_type_); }

  std::type_index getType() const final { return std::type_index(typeid(T)); }

  void* recover() final { return &any_type_; }

  const void* recover() const final { return &any_type_; }

  bool operator==(const AnyInnerBase& rhs) const final
  {
    // Compare class types before casting the incoming object to the T type
    if (rhs.getType() == getType())
    {
      auto any_type = static_cast<const T*>(rhs.recover());
      return any_type_ == *any_type;
    }
    return false;
  }

  bool operator!=(const AnyInnerBase& rhs) const final { return !operator==(rhs); }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    // If this line is removed a exception is thrown for unregistered cast need to too look into this.
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<AnyInnerBase>(*this));
    ar& boost::serialization::make_nvp("impl", any_type_);
  }

  T any_type_;
};
}  // namespace detail_any
#endif  // SWIG
}  // namespace tesseract_common

namespace boost
{
// Taken from pagmo to address the same issue
// NOTE: in some earlier versions of Boost (i.e., at least up to 1.67)
// the is_virtual_base_of type trait, used by the Boost serialization library, fails
// with a compile time error if a class is declared final. Thus, we provide a specialised
// implementation of this type trait to work around the issue. See:
// https://www.boost.org/doc/libs/1_52_0/libs/type_traits/doc/html/boost_typetraits/reference/is_virtual_base_of.html
// https://stackoverflow.com/questions/18982064/boost-serialization-of-base-class-of-final-subclass-error
// We never use virtual inheritance, thus the specialisation is always false.
template <typename T>
struct is_virtual_base_of<tesseract_common::detail_any::AnyInnerBase, tesseract_common::detail_any::AnyInner<T>>
  : false_type
{
};
}  // namespace boost

namespace tesseract_common
{
class Any
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Any, uncvref_t<T>>::value, int>;

public:
  template <typename T, generic_ctor_enabler<T> = 0>
  Any(T&& any_type)  // NOLINT
    : any_type_(std::make_unique<detail_any::AnyInner<uncvref_t<T>>>(any_type))
  {
  }

  Any()  // NOLINT
    : any_type_(nullptr)
  {
  }

  // Destructor
  ~Any() = default;

  // Copy constructor
  Any(const Any& other) : any_type_(other.any_type_->clone()) {}

  // Move ctor.
  Any(Any&& other) noexcept { any_type_.swap(other.any_type_); }

  // Move assignment.
  Any& operator=(Any&& other) noexcept
  {
    any_type_.swap(other.any_type_);
    return (*this);
  }

  // Copy assignment.
  Any& operator=(const Any& other)
  {
    (*this) = Any(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Any& operator=(T&& other)
  {
    (*this) = Any(std::forward<T>(other));
    return (*this);
  }

  std::type_index getType() const
  {
    if (any_type_ == nullptr)
      return std::type_index(typeid(nullptr));

    return any_type_->getType();
  }

  bool operator==(const Any& rhs) const { return any_type_->operator==(*rhs.any_type_); }

  bool operator!=(const Any& rhs) const { return !operator==(rhs); }

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::bad_cast();

    auto p = static_cast<uncvref_t<T>*>(any_type_->recover());
    return *p;
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::bad_cast();

    auto p = static_cast<const uncvref_t<T>*>(any_type_->recover());
    return *p;
  }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& boost::serialization::make_nvp("any_type", any_type_);
  }

  std::unique_ptr<detail_any::AnyInnerBase> any_type_;
};

}  // namespace tesseract_common

BOOST_CLASS_TRACKING(tesseract_common::Any, boost::serialization::track_never);
#endif  // TESSERACT_COMMON_ANY_H
