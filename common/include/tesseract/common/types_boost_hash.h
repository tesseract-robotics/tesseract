/**
 * @file types_boost_hash.h
 * @brief Boost hashing support for NameId and OrderedIdPair
 *
 * @author Roelof Oomen
 * @date August 4, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_TYPES_BOOST_HASH_H
#define TESSERACT_COMMON_TYPES_BOOST_HASH_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/version.hpp>
#include <boost/container_hash/hash_fwd.hpp>
#if BOOST_VERSION >= 108100
#include <boost/unordered/hash_traits.hpp>
/** @brief Defined iff this header declares the boost hashes avalanching; boost::unordered::hash_traits is older. */
#define TESSERACT_COMMON_HAS_HASH_IS_AVALANCHING
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <cstddef>

#include <tesseract/common/types.h>

// ---------------------------------------------------------------------------
// Include this header to hash NameId or OrderedIdPair with boost::hash, either
// directly or through anything that composes it (boost::hash_combine, the
// std::pair/tuple/container overloads, boost's unordered containers).
//
// The ADL hooks and the hash_is_avalanching specialization must stay in one
// header; neither belongs in types.h on its own. boost::unordered picks its
// mix policy from hash_is_avalanching at container instantiation, so a
// translation unit that saw only the hooks would give the same container type
// a different bucket placement than one that also saw the trait, and the two
// would disagree about where a key lives.
// ---------------------------------------------------------------------------

namespace tesseract::common
{
/** @brief ADL hook for boost::hash. Returns the same value as std::hash<NameId<Tag>>. */
template <typename Tag>
constexpr std::size_t hash_value(const NameId<Tag>& id) noexcept
{
  return id.value();
}

/** @brief ADL hook for boost::hash. Returns the same value as std::hash<OrderedIdPair<Tag>>. */
template <typename Tag>
std::size_t hash_value(const OrderedIdPair<Tag>& p) noexcept
{
  return p.hash();
}

}  // namespace tesseract::common

#ifdef TESSERACT_COMMON_HAS_HASH_IS_AVALANCHING
namespace boost::unordered
{
/**
 * @brief The id value is boost::hash<std::string> output, which boost itself declares avalanching.
 * @details Boost gained this trait in 1.81; on older boost the containers that consult it do not
 *          exist yet, so its absence costs nothing.
 */
template <typename Tag>
struct hash_is_avalanching<boost::hash<tesseract::common::NameId<Tag>>> : boost::true_type
{
};

/** @brief The pair hash is boost::hash_combine output, avalanching whatever its inputs were. */
template <typename Tag>
struct hash_is_avalanching<boost::hash<tesseract::common::OrderedIdPair<Tag>>> : boost::true_type
{
};

}  // namespace boost::unordered
#endif

#endif  // TESSERACT_COMMON_TYPES_BOOST_HASH_H
