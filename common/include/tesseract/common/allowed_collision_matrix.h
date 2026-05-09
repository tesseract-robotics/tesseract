#ifndef TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H
#define TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <Eigen/Core>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>

namespace tesseract::common
{
/** @brief Value stored in each ACM entry — names for serialization/display, reason for the allowance. */
struct ACMEntry
{
  std::string name1;
  std::string name2;
  std::string reason;

  bool operator==(const ACMEntry& other) const
  {
    const bool names_match =
        (name1 == other.name1 && name2 == other.name2) || (name1 == other.name2 && name2 == other.name1);
    return names_match && reason == other.reason;
  }
  bool operator!=(const ACMEntry& other) const { return !(*this == other); }
};

using AllowedCollisionEntries = std::unordered_map<LinkIdPair, ACMEntry>;

bool operator==(const AllowedCollisionEntries& entries_1, const AllowedCollisionEntries& entries_2);

class AllowedCollisionMatrix;

template <class Archive>
void save(Archive& ar, const AllowedCollisionMatrix& obj);
template <class Archive>
void load(Archive& ar, AllowedCollisionMatrix& obj);

class AllowedCollisionMatrix
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<AllowedCollisionMatrix>;
  using ConstPtr = std::shared_ptr<const AllowedCollisionMatrix>;

  AllowedCollisionMatrix() = default;
  AllowedCollisionMatrix(const AllowedCollisionEntries& entries);
  virtual ~AllowedCollisionMatrix() = default;
  AllowedCollisionMatrix(const AllowedCollisionMatrix&) = default;
  AllowedCollisionMatrix& operator=(const AllowedCollisionMatrix&) = default;
  AllowedCollisionMatrix(AllowedCollisionMatrix&&) = default;
  AllowedCollisionMatrix& operator=(AllowedCollisionMatrix&&) = default;

  /**
   * @brief Disable collision between two collision objects
   * @param link_id1 Collision object LinkId
   * @param link_id2 Collision object LinkId
   * @param reason The reason for disabling collision
   */
  virtual void addAllowedCollision(const LinkId& link_id1, const LinkId& link_id2, const std::string& reason);

  /**
   * @brief Disable collision for a pre-built canonical pair, supplying the entry's names + reason.
   * @details Useful when iterating an existing @ref AllowedCollisionEntries map and re-inserting
   *          {key, entry} pairs without reconstructing @ref LinkId objects from strings.
   *          Inserts the entry, or — if @p pair is already present with matching names —
   *          updates the reason. Throws std::runtime_error if @p pair is already present with
   *          different names (hash collision).
   * @param pair Canonically ordered link-id pair (the map key)
   * @param entry The ACM entry value (names + reason)
   */
  virtual void addAllowedCollision(const LinkIdPair& pair, const ACMEntry& entry);

  /**
   * @brief Get all of the entries in the allowed collision matrix
   * @return AllowedCollisionEntries keyed by LinkIdPair with ACMEntry values containing names and reason
   */
  const AllowedCollisionEntries& getAllAllowedCollisions() const;

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param link_id1 Collision object LinkId
   * @param link_id2 Collision object LinkId
   */
  virtual void removeAllowedCollision(const LinkId& link_id1, const LinkId& link_id2);

  /**
   * @brief Remove disabled collision pair from allowed collision matrix using a pre-built canonical pair.
   * @details Useful when iterating an existing @ref AllowedCollisionEntries map and erasing
   *          by key without reconstructing @ref LinkId objects from strings.
   * @param pair Canonically ordered link-id pair (the map key)
   */
  virtual void removeAllowedCollision(const LinkIdPair& pair);

  /**
   * @brief Remove disabled collision for any pair with link_id from allowed collision matrix
   * @param link_id Collision object LinkId
   */
  virtual void removeAllowedCollision(const LinkId& link_id);

  /**
   * @brief This checks if a link pair is allowed to be in collision (hot-path primary).
   * @param pair Canonically ordered link-id pair
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const LinkIdPair& pair) const;

  /**
   * @brief Convenience overload; forwards to the pair-based primary.
   */
  bool isCollisionAllowed(const LinkId& link_id1, const LinkId& link_id2) const
  {
    return isCollisionAllowed(LinkIdPair(link_id1, link_id2));
  }

  /**
   * @brief Clears the list of allowed collisions, so that no collision will be
   *        allowed.
   */
  void clearAllowedCollisions();

  /**
   * @brief Inserts an allowable collision matrix ignoring duplicate pairs
   * @param acm ACM to be inserted
   */
  void insertAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

  /**
   * @brief Reserve space for the internal data storage
   * @param size The size to reserve
   */
  void reserveAllowedCollisionMatrix(std::size_t size);

  bool operator==(const AllowedCollisionMatrix& rhs) const;
  bool operator!=(const AllowedCollisionMatrix& rhs) const;

private:
  AllowedCollisionEntries lookup_table_;

  /**
   * @brief Insert an entry or, if the key already exists, verify the stored names match
   *        (throwing via checkPairHashCollision otherwise) and update the reason.
   *        Single write path used by every mutation entry point.
   */
  void insertEntryChecked(const LinkIdPair& key, ACMEntry entry);

  template <class Archive>
  friend void ::tesseract::common::save(Archive& ar, const AllowedCollisionMatrix& obj);
  template <class Archive>
  friend void ::tesseract::common::load(Archive& ar, AllowedCollisionMatrix& obj);
};

std::ostream& operator<<(std::ostream& os, const AllowedCollisionMatrix& acm);
}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H
