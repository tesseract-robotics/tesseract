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

#include <string>
#include <unordered_map>

#define COMMA ,

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_ANY_EXPORT_KEY(C, K)                                                                                 \
  namespace tesseract_serialization::any_poly                                                                          \
  {                                                                                                                    \
  using K##AnyInstanceBase = tesseract_common::TypeErasureInstance<C, tesseract_common::TypeErasureInterface>;         \
  using K##AnyInstance = tesseract_common::detail_any::AnyInstance<C>;                                                 \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(tesseract_serialization::any_poly::K##AnyInstanceBase)                                        \
  BOOST_CLASS_EXPORT_KEY(tesseract_serialization::any_poly::K##AnyInstance)                                            \
  BOOST_CLASS_TRACKING(tesseract_serialization::any_poly::K##AnyInstanceBase, boost::serialization::track_never)       \
  BOOST_CLASS_TRACKING(tesseract_serialization::any_poly::K##AnyInstance, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_ANY_EXPORT_IMPLEMENT(K)                                                                              \
  BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_serialization::any_poly::K##AnyInstanceBase)                                  \
  BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_serialization::any_poly::K##AnyInstance)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_ANY_EXPORT(C, K)                                                                                     \
  TESSERACT_ANY_EXPORT_KEY(C, K)                                                                                       \
  TESSERACT_ANY_EXPORT_IMPLEMENT(K)

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
    UNUSED(assign);
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

  std::unique_ptr<tesseract_common::TypeErasureInterface> clone() const final
  {
    return std::make_unique<AnyInstance<T>>(this->get());
  }

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

TESSERACT_ANY_EXPORT_KEY(bool, IntegralBool)
TESSERACT_ANY_EXPORT_KEY(int, IntegralInt)
TESSERACT_ANY_EXPORT_KEY(unsigned, IntegralUnsigned)
TESSERACT_ANY_EXPORT_KEY(double, IntegralDouble)
TESSERACT_ANY_EXPORT_KEY(float, IntegralFloat)
TESSERACT_ANY_EXPORT_KEY(std::string, StdString)
TESSERACT_ANY_EXPORT_KEY(std::size_t, StdSizeT)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA std::string>, StdUnorderedMapStringString)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA bool>, StdUnorderedMapStringBool)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA int>, StdUnorderedMapStringInt)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA unsigned>, StdUnorderedMapStringUnsigned)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA double>, StdUnorderedMapStringDouble)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA float>, StdUnorderedMapStringFloat)
TESSERACT_ANY_EXPORT_KEY(std::unordered_map<std::string COMMA std::size_t>, StdUnorderedMapStringStdSizeT)

#endif  // TESSERACT_COMMON_ANY_POLY_H
