/**
 * @file serialization_extensions.h
 * @brief Boost serialization class extension macros and helpers
 *
 * @author Levi Armstrong
 * @date July 24, 2022
 * @version TODO
 * @bug No known bugs
 *
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
#ifndef TESSERACT_COMMON_SERIALIZATION_EXTENSIONS_H
#define TESSERACT_COMMON_SERIALIZATION_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/mpl/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/sfinae_utils.h>

CREATE_MEMBER_CHECK(extension_type);

namespace tesseract_common
{
namespace serialization::xml
{
template <class T>
struct extension
{
  template <class U>
  struct traits_class_extension
  {
    using extension_type = typename U::extenstion;
  };

  using extension_type = typename boost::mpl::
      eval_if<has_member_extension_type<T>, traits_class_extension<T>, boost::mpl::string<'t', 'r', 's', 'x'>>::type;

  static constexpr const char* value = boost::mpl::c_str<extension::extension_type>::value;
};
}  // namespace serialization::xml

namespace serialization::binary
{
template <class T>
struct extension
{
  template <class U>
  struct traits_class_extension
  {
    using extension_type = typename U::extenstion;
  };

  using extension_type = typename boost::mpl::
      eval_if<has_member_extension_type<T>, traits_class_extension<T>, boost::mpl::string<'t', 'r', 's', 'b'>>::type;

  static constexpr const char* value = boost::mpl::c_str<extension::extension_type>::value;
};
}  // namespace serialization::binary
}  // namespace tesseract_common

/**
 * @brief A macro for defining serialization extension for classes
 * @param T the class to define extensions for
 * @param X the xml serialziation extension for the provided class
 * @param B the binary serialzation extension for the provided class
 */
#define TESSERACT_CLASS_EXTENSION(T, X, B)                                                                             \
  namespace tesseract_common                                                                                           \
  {                                                                                                                    \
  namespace serialization::xml                                                                                         \
  {                                                                                                                    \
  template <>                                                                                                          \
  struct extension<T>                                                                                                  \
  {                                                                                                                    \
    static constexpr const char* value = X;                                                                            \
  };                                                                                                                   \
  }                                                                                                                    \
  namespace serialization::binary                                                                                      \
  {                                                                                                                    \
  template <>                                                                                                          \
  struct extension<T>                                                                                                  \
  {                                                                                                                    \
    static constexpr const char* value = B;                                                                            \
  };                                                                                                                   \
  }                                                                                                                    \
  }

#endif  // TESSERACT_COMMON_SERIALIZATION_EXTENSIONS_H
