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
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/concept_check.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/type_erasure.h>

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_ANY_EXPORT_KEY(N, C)                                                                                 \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##AnyInstanceBase = tesseract_common::TypeErasureInstance<C, tesseract_common::TypeErasureInterface>;         \
  using C##AnyInstance = tesseract_common::detail_any::AnyInstance<C>;                                                 \
  using C##AnyInstanceWrapper = tesseract_common::TypeErasureInstanceWrapper<C##AnyInstance>;                          \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##AnyInstanceBase)                                                                        \
  BOOST_CLASS_EXPORT_KEY(N::C##AnyInstance)                                                                            \
  BOOST_CLASS_EXPORT_KEY(N::C##AnyInstanceWrapper)                                                                     \
  BOOST_CLASS_TRACKING(N::C##AnyInstanceBase, boost::serialization::track_never)                                       \
  BOOST_CLASS_TRACKING(N::C##AnyInstance, boost::serialization::track_never)                                           \
  BOOST_CLASS_TRACKING(N::C##AnyInstanceWrapper, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_ANY_EXPORT_IMPLEMENT(inst)                                                                           \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##AnyInstanceBase)                                                                  \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##AnyInstance)                                                                      \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##AnyInstanceWrapper)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_ANY_EXPORT(N, C)                                                                                     \
  TESSERACT_ANY_EXPORT_KEY(N, C)                                                                                       \
  TESSERACT_ANY_EXPORT_IMPLEMENT(N::C)

namespace tesseract_common::detail_any
{
template <typename A>
struct AnyConcept  // NOLINT
  : boost::Assignable<A>,
    boost::CopyConstructible<A>,
    boost::EqualityComparable<A>
{
  BOOST_CONCEPT_USAGE(AnyConcept)
  {
    A cp(c);
    A assign = c;
    bool eq = (c == cp);
    bool neq = (c != cp);
    UNUSED(eq);
    UNUSED(neq);
  }

private:
  A c;
};

template <typename T>
struct AnyInstance : tesseract_common::TypeErasureInstance<T, tesseract_common::TypeErasureInterface>  // NOLINT
{
  using BaseType = tesseract_common::TypeErasureInstance<T, tesseract_common::TypeErasureInterface>;
  AnyInstance() = default;
  AnyInstance(const T& x) : BaseType(x) {}
  AnyInstance(AnyInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((AnyConcept<T>));

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<BaseType>(*this));
  }
};
}  // namespace tesseract_common::detail_any

namespace tesseract_common
{
using AnyPolyBase = tesseract_common::TypeErasureBase<TypeErasureInterface, detail_any::AnyInstance>;

struct AnyPoly : AnyPolyBase
{
  using AnyPolyBase::AnyPolyBase;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_common

BOOST_CLASS_EXPORT_KEY(tesseract_common::AnyPolyBase)
BOOST_CLASS_TRACKING(tesseract_common::AnyPolyBase, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_common::AnyPoly)
BOOST_CLASS_TRACKING(tesseract_common::AnyPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMON_ANY_POLY_H
