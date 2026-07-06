/**
 * @file types.h
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_TYPES_H
#define TESSERACT_COMMON_TYPES_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// ---------------------------------------------------------------------------
// Implicit-conversion toggle for NameId string constructors.
//
// By default the constructors taking std::string / const char* are implicit,
// which is convenient for tests and downstream users. Tesseract's own
// production libraries set TESSERACT_NAMEID_NO_IMPLICIT (via target_compile_-
// definitions PRIVATE) to make the constructors explicit in their own TUs,
// forcing internal call sites to either avoid string-keyed paths entirely or
// be explicit about when they construct an id from a string. Templates are
// instantiated per-TU, so the differing explicit-ness across TUs is a
// compile-time check only — it has no ABI/ODR impact.
// ---------------------------------------------------------------------------
#ifdef TESSERACT_NAMEID_NO_IMPLICIT
#define TESSERACT_NAMEID_EXPLICIT explicit
#else
#define TESSERACT_NAMEID_EXPLICIT
#endif

namespace tesseract::common
{
// ---------------------------------------------------------------------------
// Integer link/joint identity types
// ---------------------------------------------------------------------------

/**
 * @brief Underlying numeric type for NameId hash values.
 *
 * Aliased so the width can be widened later without churning every public API
 * and downstream caller. Note: hashes are produced by std::hash<std::string>,
 * which returns std::size_t, so a wider NameIdValue alone does not widen the
 * hash — a different hash function would also be required.
 */
using NameIdValue = std::uint64_t;

struct LinkTag
{
};
struct JointTag
{
};

/**
 * @brief Tagged integer identity type for links and joints.
 *
 * Wraps a NameIdValue computed from the name via std::hash<std::string>.
 * Distinct tag types (LinkTag, JointTag) prevent accidental cross-use.
 * IDs are runtime-only — never persisted. Deterministic within a single
 * process execution.
 */
template <typename Tag>
struct NameId
{
  NameId() = default;

  // NOLINTNEXTLINE(google-explicit-constructor)
  TESSERACT_NAMEID_EXPLICIT NameId(const std::string& name)
  {
    if (!name.empty())
    {
      auto h = static_cast<NameIdValue>(std::hash<std::string>{}(name));
      value_ = (h == 0) ? NameIdValue{ 1 } : h;
      name_ = name;
    }
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  TESSERACT_NAMEID_EXPLICIT NameId(const char* name) : NameId(name != nullptr ? std::string(name) : std::string{}) {}

  /** @brief The numeric hash of the name. Zero means invalid/default-constructed. */
  [[nodiscard]] constexpr NameIdValue value() const noexcept { return value_; }

  /** @brief Access the original name string. Empty for default-constructed (invalid) IDs. */
  [[nodiscard]] const std::string& name() const noexcept { return name_; }

  [[nodiscard]] constexpr bool isValid() const noexcept { return value_ != 0; }

  /**
   * @brief Watertight (hybrid) equality: compare the cached hash value first; only when the
   *        values match, confirm with the name strings. Two distinct names that collide on the
   *        64-bit hash therefore compare UNEQUAL, so hash-keyed containers keep them as distinct
   *        keys and resolve the collision exactly like a string-keyed hash map would.
   * @details Load-bearing invariant: every valid NameId carries the name it was constructed
   *          from (the string constructors are the only public way to obtain a valid id), so
   *          the name is always available for the confirming compare.
   */
  bool operator==(const NameId& other) const noexcept { return value_ == other.value_ && name_ == other.name_; }
  bool operator!=(const NameId& other) const noexcept { return !(*this == other); }

  /**
   * @brief Order by hash value, NOT by name — hash-random, useful for ordered containers but
   *        produces no human-meaningful ordering (sort by name() explicitly for display).
   *        Value ties (hash collisions) are broken by name so ordering is consistent with
   *        operator== (a strict weak ordering whose equivalence is exactly hybrid equality).
   */
  bool operator<(const NameId& other) const noexcept
  {
    if (value_ != other.value_)
      return value_ < other.value_;
    return name_ < other.name_;
  }

  /**
   * @brief Test-only factory: construct an id with an explicit value/name combination.
   * @details Exists so unit tests can manufacture hash collisions (two names sharing one
   *          value), which cannot be produced from real strings on demand. Never use outside
   *          tests: an id whose value was not derived from its name breaks the "value is a
   *          function of the name" property every container in the framework relies on.
   */
  [[nodiscard]] static NameId createWithValueForTesting(NameIdValue value, std::string name)
  {
    NameId id;
    id.value_ = value;
    id.name_ = std::move(name);
    return id;
  }

private:
  NameIdValue value_{ 0 };
  std::string name_;
};

using LinkId = NameId<LinkTag>;
using JointId = NameId<JointTag>;

inline const LinkId INVALID_LINK_ID{};
inline const JointId INVALID_JOINT_ID{};

/**
 * @brief Canonically ordered pair of ids with cached combined hash — the key type for
 *        pair-keyed maps (ACM, collision margins, collision coefficients).
 *
 * Stores the two full NameIds (values AND names) so pair equality can confirm names whenever
 * the 64-bit values match — the same hybrid scheme as NameId::operator==. This makes every
 * pair-keyed container watertight against hash collisions: colliding pairs compare unequal
 * and coexist as distinct keys, exactly as in a string-keyed hash map.
 *
 * Canonicalization orders by (value, then name on value ties), so OrderedIdPair(a, b) ==
 * OrderedIdPair(b, a) holds even when a and b collide on the hash value.
 */
template <typename Tag>
struct OrderedIdPair
{
  OrderedIdPair() = default;
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b)
    : OrderedIdPair(a, b, (a.value() < b.value()) || (a.value() == b.value() && a.name() <= b.name()))
  {
  }

  [[nodiscard]] const NameId<Tag>& first() const noexcept { return first_; }
  [[nodiscard]] const NameId<Tag>& second() const noexcept { return second_; }
  [[nodiscard]] NameIdValue first_id() const noexcept { return first_.value(); }
  [[nodiscard]] NameIdValue second_id() const noexcept { return second_.value(); }
  [[nodiscard]] std::size_t hash() const noexcept { return hash_; }

  bool operator==(const OrderedIdPair& other) const noexcept
  {
    return first_ == other.first_ && second_ == other.second_;  // hybrid via NameId::operator==
  }
  bool operator!=(const OrderedIdPair& other) const noexcept { return !(*this == other); }

  bool operator<(const OrderedIdPair& other) const noexcept
  {
    if (first_ != other.first_)
      return first_ < other.first_;
    return second_ < other.second_;
  }

  /** @brief Mix two id values into one bucket hash (shared with OrderedIdPairView lookups). */
  static std::size_t combineHash(NameIdValue f, NameIdValue s) noexcept
  {
    auto h = static_cast<std::size_t>(f);
    h ^= static_cast<std::size_t>(s) + static_cast<std::size_t>(0x9e3779b97f4a7c15ULL) + (h << 6) + (h >> 2);
    return h;
  }

private:
  /** @brief Delegation target: canonical order already decided, so members initialize directly. */
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b, bool a_first)
    : first_(a_first ? a : b), second_(a_first ? b : a), hash_(combineHash(first_.value(), second_.value()))
  {
  }

  NameId<Tag> first_;
  NameId<Tag> second_;
  std::size_t hash_{ 0 };
};

using LinkIdPair = OrderedIdPair<LinkTag>;

static_assert(sizeof(LinkIdPair) == (2 * sizeof(LinkId)) + sizeof(std::size_t),
              "LinkIdPair is two full NameIds plus the cached pair hash");

/** @brief ADL hook for boost::hash and boost-hashed containers (e.g. boost::unordered_flat_map). */
template <typename Tag>
constexpr std::size_t hash_value(const NameId<Tag>& id) noexcept
{
  return static_cast<std::size_t>(id.value());
}

/**
 * @brief ADL hook for boost::hash and boost-hashed containers (e.g. boost::unordered_flat_map).
 * @details Not constexpr: it calls the non-constexpr OrderedIdPair::hash() accessor. A constexpr
 *          function that can never produce a constant expression is ill-formed, no diagnostic
 *          required — some compilers (e.g. MSVC) reject it outright.
 */
template <typename Tag>
std::size_t hash_value(const OrderedIdPair<Tag>& p) noexcept
{
  return p.hash();
}

/** @brief Convert a vector of strings to a vector of NameId<Tag> */
template <typename IdT>
inline std::vector<IdT> toIds(const std::vector<std::string>& names)
{
  std::vector<IdT> ids;
  ids.reserve(names.size());
  for (const auto& n : names)
    ids.emplace_back(n);
  return ids;
}

/** @brief Convert any container of NameId<Tag> to a vector of name strings */
template <typename Container>
inline std::vector<std::string> toNames(const Container& ids)
{
  std::vector<std::string> names;
  names.reserve(ids.size());
  for (const auto& id : ids)
    names.push_back(id.name());
  return names;
}

}  // namespace tesseract::common

// std::hash specializations
namespace std
{
/** @brief Identity hash for NameId types — enables use with std::unordered_map/set. */
template <typename Tag>
struct hash<tesseract::common::NameId<Tag>>
{
  constexpr std::size_t operator()(const tesseract::common::NameId<Tag>& id) const noexcept
  {
    return static_cast<std::size_t>(id.value());
  }
};

template <typename Tag>
struct hash<tesseract::common::OrderedIdPair<Tag>>
{
  // Not constexpr: OrderedIdPair::hash() is not constexpr (see the hash_value ADL hook above).
  std::size_t operator()(const tesseract::common::OrderedIdPair<Tag>& p) const noexcept { return p.hash(); }
};

}  // namespace std

#endif  // TESSERACT_COMMON_TYPES_H
