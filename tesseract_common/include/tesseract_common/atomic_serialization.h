/**
 * @file atomic_serialization.h
 * @brief Additional Boost serialization wrappers
 * @details Support for atomic serialization
 *
 * @author Levi Armstrong
 * @date March 24, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_COMMON_ATOMIC_SERIALIZATION_H
#define TESSERACT_COMMON_ATOMIC_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

/** @note When using this header only include it in the cpp never in the header */
namespace boost::serialization
{
template <class Archive, class T>
inline void save(Archive& ar, const std::atomic<T>& t, const unsigned int)
{
  const T value = t.load();
  ar& BOOST_SERIALIZATION_NVP(value);
}

template <class Archive, class T>
inline void load(Archive& ar, std::atomic<T>& t, const unsigned int)
{
  T value;
  ar& BOOST_SERIALIZATION_NVP(value);
  t = value;
}

template <class Archive, class T>
inline void serialize(Archive& ar, std::atomic<T>& t, const unsigned int version)
{
  boost::serialization::split_free(ar, t, version);
}
}  // namespace boost::serialization
#endif  // TESSERACT_COMMON_ATOMIC_SERIALIZATION_H
