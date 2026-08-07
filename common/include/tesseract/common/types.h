/**
 * @file types.h
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @author Roelof Oomen
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
#include <cstddef>
#include <functional>
#include <iosfwd>
#include <string>
#include <type_traits>
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
// instantiated per-TU, so the check is compile-time only. Formally the
// differing token sequences across TUs are an ODR violation (ill-formed, no
// diagnostic required) — a deliberate, benign one: explicit affects overload
// resolution only, not layout or codegen. C++20 modules would reject it.
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
 * @brief Underlying numeric type for NameId hash values, as produced by boost::hash<std::string>.
 *
 * std::size_t, the width every standard and boost hashed container reduces a hash to. Naming it
 * separately marks the values that are raw name hashes — they are runtime-only and must never be
 * persisted (see the cereal serializers, which archive names).
 */
using NameIdValue = std::size_t;

/**
 * @brief Hash a name into a NameIdValue.
 * @details boost::hash<std::string>, which is avalanching and identical on every platform. Defined
 *          out of line so this header stays free of boost includes.
 */
[[nodiscard]] NameIdValue nameIdHash(const std::string& name) noexcept;

struct LinkTag
{
};
struct JointTag
{
};

/**
 * @brief Tagged integer identity type for links and joints.
 *
 * Wraps a NameIdValue computed from the name via boost::hash<std::string>.
 * Distinct tag types (LinkTag, JointTag) prevent accidental cross-use.
 * IDs are runtime-only — never persisted. Deterministic within a single
 * process execution.
 */
template <typename Tag>
struct NameId
{
  NameId() = default;

  // NOLINTNEXTLINE(google-explicit-constructor)
  TESSERACT_NAMEID_EXPLICIT NameId(std::string name) noexcept
  {
    if (!name.empty())
    {
      value_ = nameIdHash(name);
      name_ = std::move(name);
    }
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  TESSERACT_NAMEID_EXPLICIT NameId(const char* name) : NameId(name != nullptr ? std::string(name) : std::string{}) {}

  /**
   * @brief The numeric hash of the name. Zero for an invalid id, but zero is not reserved — a name
   *        may legitimately hash to it, so validity is decided by the name, not by this value.
   */
  [[nodiscard]] constexpr NameIdValue value() const noexcept { return value_; }

  /** @brief Access the original name string. Empty for default-constructed (invalid) IDs. */
  [[nodiscard]] const std::string& name() const noexcept { return name_; }

  /** @brief An id is valid iff it carries a name; the string constructors are the only source of one. */
  [[nodiscard]] bool isValid() const noexcept { return !name_.empty(); }

  /**
   * @brief Watertight (hybrid) equality: compare the cached hash value first; only when the
   *        values match, confirm with the name strings. Two distinct names that collide on the
   *        hash value therefore compare UNEQUAL, so hash-keyed containers keep them as distinct
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

private:
  /**
   * @brief Test-only backdoor for manufacturing hash collisions (ids whose value is not derived from the name).
   */
  friend struct NameIdTestAccess;

  NameIdValue value_{ 0 };
  std::string name_;
};

using LinkId = NameId<LinkTag>;
using JointId = NameId<JointTag>;

inline const LinkId INVALID_LINK_ID{};
inline const JointId INVALID_JOINT_ID{};

/**
 * @brief Mix two id values into one bucket hash — the hash of the OrderedIdPair holding them, so a
 *        caller can compute a pair's bucket hash without constructing one.
 * @details boost::hash_combine over the two values. Defined out of line so this header stays free
 *          of boost includes.
 */
[[nodiscard]] std::size_t combineNameIdHash(NameIdValue f, NameIdValue s) noexcept;

/**
 * @brief Canonically ordered pair of ids with cached combined hash — the key type for
 *        pair-keyed maps (ACM, collision margins, collision coefficients).
 *
 * Stores the two full NameIds (values AND names) so pair equality can confirm names whenever
 * the hash values match — the same hybrid scheme as NameId::operator==. This makes every
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

  /**
   * @brief In-place equivalent of *this = OrderedIdPair(a, b): re-canonicalizes and reuses the
   *        existing ids' string capacity, so reassigning a long-lived pair performs no allocation
   *        once capacity suffices.
   */
  void assign(const NameId<Tag>& a, const NameId<Tag>& b)
  {
    if (&a == &first_ || &a == &second_ || &b == &first_ || &b == &second_)
    {
      *this = OrderedIdPair(a, b);
      return;
    }

    const bool a_first = (a.value() < b.value()) || (a.value() == b.value() && a.name() <= b.name());
    first_ = a_first ? a : b;
    second_ = a_first ? b : a;
    hash_ = combineNameIdHash(first_.value(), second_.value());
  }

  [[nodiscard]] const NameId<Tag>& first() const noexcept { return first_; }
  [[nodiscard]] const NameId<Tag>& second() const noexcept { return second_; }
  [[nodiscard]] std::size_t hash() const noexcept { return hash_; }

  /**
   * @brief References to the two names, lexicographically smaller first.
   * @details first() and second() order by hash value, which is a runtime property and not
   *          alphabetical. Anything writing the pair somewhere its order is observable — an
   *          archive, a document, a file — must use this instead, or the output depends on the
   *          hashing implementation.
   *
   *          The result references this pair's names and does not own them. Besides the usual
   *          rule that it must not outlive the pair, assign() re-canonicalizes in place and
   *          reuses the ids' string capacity, so a result taken before an assign() is invalidated
   *          by it even though the pair itself is still alive. Copy the names if either applies.
   */
  [[nodiscard]] std::pair<const std::string&, const std::string&> orderedNameView() const& noexcept
  {
    // std::minmax, spelled out so types.h does not pull <algorithm> into every consumer.
    using View = std::pair<const std::string&, const std::string&>;
    return (first_.name() <= second_.name()) ? View(first_.name(), second_.name()) :
                                               View(second_.name(), first_.name());
  }
  std::pair<const std::string&, const std::string&> orderedNameView() const&& = delete;

  /** @brief Hybrid equality: both values are compared first, the names confirm on matches. */
  bool operator==(const OrderedIdPair& other) const noexcept
  {
    return first_.value() == other.first_.value() && second_.value() == other.second_.value() &&
           first_.name() == other.first_.name() && second_.name() == other.second_.name();
  }
  bool operator!=(const OrderedIdPair& other) const noexcept { return !(*this == other); }

  /** @brief Lexicographic by (first, second) under NameId ordering; consistent with operator==. */
  bool operator<(const OrderedIdPair& other) const noexcept
  {
    if (first_.value() != other.first_.value())
      return first_.value() < other.first_.value();
    if (const int cmp = first_.name().compare(other.first_.name()); cmp != 0)
      return cmp < 0;
    if (second_.value() != other.second_.value())
      return second_.value() < other.second_.value();
    return second_.name() < other.second_.name();
  }

private:
  /** @brief Delegation target: canonical order already decided, so members initialize directly. */
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b, bool a_first)
    : first_(a_first ? a : b), second_(a_first ? b : a), hash_(combineNameIdHash(first_.value(), second_.value()))
  {
  }

  NameId<Tag> first_;
  NameId<Tag> second_;
  std::size_t hash_{ combineNameIdHash(0, 0) };
};

using LinkIdPair = OrderedIdPair<LinkTag>;

static_assert(sizeof(LinkIdPair) == (2 * sizeof(LinkId)) + sizeof(std::size_t),
              "LinkIdPair is two full NameIds plus the cached pair hash");

/** @brief Stream insertion: writes exactly id.name(). Print value() explicitly where the numeric id matters. */
template <typename Tag>
std::ostream& operator<<(std::ostream& os, const NameId<Tag>& id)
{
  return os << id.name();
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

// std::hash specializations — enables use with std::unordered_map/set.
// For boost::hash, boost::hash_combine and boost's unordered containers, include types_boost_hash.h.
namespace std
{
/** @brief Hash for NameId — returns the cached hash value; no rehash per lookup. */
template <typename Tag>
struct hash<tesseract::common::NameId<Tag>>
{
  using is_avalanching = std::true_type;  // Instruct Boost.Unordered to not use post-mixing

  constexpr std::size_t operator()(const tesseract::common::NameId<Tag>& id) const noexcept
  {
    return static_cast<std::size_t>(id.value());
  }
};

/** @brief Hash for OrderedIdPair — returns the cached combined hash; no rehash per lookup. */
template <typename Tag>
struct hash<tesseract::common::OrderedIdPair<Tag>>
{
  using is_avalanching = std::true_type;  // Instruct Boost.Unordered to not use post-mixing

  std::size_t operator()(const tesseract::common::OrderedIdPair<Tag>& p) const noexcept { return p.hash(); }
};

}  // namespace std

#endif  // TESSERACT_COMMON_TYPES_H
