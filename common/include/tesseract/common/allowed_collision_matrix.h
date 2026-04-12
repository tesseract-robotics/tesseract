#ifndef TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H
#define TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <Eigen/Core>
#include <unordered_map>
#include <tesseract/common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

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
    return name1 == other.name1 && name2 == other.name2 && reason == other.reason;
  }
  bool operator!=(const ACMEntry& other) const { return !(*this == other); }
};

using AllowedCollisionEntries = std::unordered_map<LinkIdPair, ACMEntry, LinkIdPair::Hash>;

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
   * @brief Disable collision between two collision objects (Tier 3 — string overload)
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collision
   */
  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  /**
   * @brief Disable collision between two collision objects (LinkId overload)
   * @param link_id1 Collision object LinkId
   * @param link_id2 Collision object LinkId
   * @param reason The reason for disabling collision
   */
  virtual void addAllowedCollision(const LinkId& link_id1, const LinkId& link_id2, const std::string& reason);

  /**
   * @brief Get all of the entries in the allowed collision matrix
   * @return AllowedCollisionEntries keyed by LinkIdPair with ACMEntry values containing names and reason
   */
  const AllowedCollisionEntries& getAllAllowedCollisions() const;

  /**
   * @brief Remove disabled collision pair from allowed collision matrix (Tier 3 — string)
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  /**
   * @brief Remove disabled collision pair from allowed collision matrix (LinkId overload)
   * @param link_id1 Collision object LinkId
   * @param link_id2 Collision object LinkId
   */
  virtual void removeAllowedCollision(const LinkId& link_id1, const LinkId& link_id2);

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name);

  /**
   * @brief Remove disabled collision for any pair with link_id from allowed collision matrix
   * @param link_id Collision object LinkId
   */
  virtual void removeAllowedCollision(const LinkId& link_id);

  /**
   * @brief This checks if two links are allowed to be in collision (Tier 1 — LinkId, hot-path)
   * @param link_id1 First link id
   * @param link_id2 Second link id
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const LinkId& link_id1, const LinkId& link_id2) const;

  /**
   * @brief This checks if two links are allowed to be in collision (Tier 3 — string)
   * @param link_name1 First link name
   * @param link_name2 Second link name
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;

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
  template <class Archive>
  friend void ::tesseract::common::save(Archive& ar, const AllowedCollisionMatrix& obj);
  template <class Archive>
  friend void ::tesseract::common::load(Archive& ar, AllowedCollisionMatrix& obj);
};

std::ostream& operator<<(std::ostream& os, const AllowedCollisionMatrix& acm);
}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H
