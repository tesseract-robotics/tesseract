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

  constexpr bool operator==(const NameId& other) const noexcept { return value_ == other.value_; }
  constexpr bool operator!=(const NameId& other) const noexcept { return value_ != other.value_; }

  /**
   * @brief Order by hash value, NOT by name. This is hash-random — useful for ordered containers
   *        (std::map / std::set keyed on NameId) but produces no human-meaningful ordering. If
   *        you need lexicographic order, sort by name() explicitly.
   */
  constexpr bool operator<(const NameId& other) const noexcept { return value_ < other.value_; }

private:
  NameIdValue value_{ 0 };
  std::string name_;
};

using LinkId = NameId<LinkTag>;
using JointId = NameId<JointTag>;

inline const LinkId INVALID_LINK_ID{};
inline const JointId INVALID_JOINT_ID{};

/**
 * @brief Canonically ordered pair of id values with cached hash.
 *
 * Holds only the two NameIdValue id values (not the source NameId names) plus a
 * cached combined hash. The constructor guarantees first_id() <= second_id()
 * regardless of argument order, so OrderedIdPair(a, b) == OrderedIdPair(b, a).
 */
template <typename Tag>
struct OrderedIdPair
{
  OrderedIdPair() = default;
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b) : OrderedIdPair(a.value(), b.value()) {}
  OrderedIdPair(NameIdValue a, NameIdValue b)
    : first_id_(a <= b ? a : b), second_id_(a <= b ? b : a), hash_(combineHash(first_id_, second_id_))
  {
  }

  [[nodiscard]] constexpr NameIdValue first_id() const noexcept { return first_id_; }
  [[nodiscard]] constexpr NameIdValue second_id() const noexcept { return second_id_; }
  [[nodiscard]] constexpr std::size_t hash() const noexcept { return hash_; }

  constexpr bool operator==(const OrderedIdPair& other) const noexcept
  {
    return first_id_ == other.first_id_ && second_id_ == other.second_id_;
  }
  constexpr bool operator!=(const OrderedIdPair& other) const noexcept { return !(*this == other); }

  constexpr bool operator<(const OrderedIdPair& other) const noexcept
  {
    if (first_id_ != other.first_id_)
      return first_id_ < other.first_id_;
    return second_id_ < other.second_id_;
  }

private:
  NameIdValue first_id_{ 0 };
  NameIdValue second_id_{ 0 };
  std::size_t hash_{ 0 };

  static constexpr std::size_t combineHash(NameIdValue f, NameIdValue s) noexcept
  {
    auto h = static_cast<std::size_t>(f);
    h ^= static_cast<std::size_t>(s) + std::size_t{ 0x9e3779b9 } + (h << 6) + (h >> 2);
    return h;
  }
};

using LinkIdPair = OrderedIdPair<LinkTag>;

static_assert(sizeof(LinkIdPair) == (2 * sizeof(NameIdValue)) + sizeof(std::size_t),
              "LinkIdPair must be two NameIdValue ids plus cached pair hash");

/** @brief Return the two names canonically ordered to match OrderedIdPair(a, b). */
template <typename Tag>
inline std::pair<std::string, std::string> orderedPairNames(const NameId<Tag>& a, const NameId<Tag>& b)
{
  return (a.value() <= b.value()) ? std::pair<std::string, std::string>{ a.name(), b.name() } :
                                    std::pair<std::string, std::string>{ b.name(), a.name() };
}

/**
 * @brief Throw if the two canonicalized names do not match the stored pair's names.
 *
 * Since NameId hashes collapse a full string into a NameIdValue (size_t-bounded, currently 64
 * bits), two distinct link names can in theory map to the same id. When that happens, every
 * LinkIdPair-keyed map silently aliases the two pairs. Call this at insertion time to surface
 * such collisions as a clear runtime error.
 */
void checkPairHashCollision(const char* context,
                            const std::string& new_name1,
                            const std::string& new_name2,
                            const std::string& existing_name1,
                            const std::string& existing_name2);

/**
 * @brief Throw if a single NameId lookup hit an existing entry whose stored name differs.
 *
 * When a NameId-keyed map finds an existing entry at the same hash as the incoming name but
 * the stored name is different, that is a genuine hash collision. Call this at insertion time
 * to surface it as a clear runtime error rather than silent aliasing.
 */
void checkHashCollision(const char* context, const std::string& new_name, const std::string& existing_name);

/** @brief ADL hook for boost::hash and boost-hashed containers (e.g. boost::unordered_flat_map). */
template <typename Tag>
constexpr std::size_t hash_value(const NameId<Tag>& id) noexcept
{
  return static_cast<std::size_t>(id.value());
}

/** @brief ADL hook for boost::hash and boost-hashed containers (e.g. boost::unordered_flat_map). */
template <typename Tag>
constexpr std::size_t hash_value(const OrderedIdPair<Tag>& p) noexcept
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
  constexpr std::size_t operator()(const tesseract::common::OrderedIdPair<Tag>& p) const noexcept { return p.hash(); }
};

}  // namespace std

#endif  // TESSERACT_COMMON_TYPES_H
