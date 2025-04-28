#ifndef TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H
#define TESSERACT_COMMON_ALLOWED_COLLISION_MATRIX_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/export.hpp>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <unordered_map>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace boost::serialization
{
class access;
}

namespace tesseract_common
{
struct Serialization;

using AllowedCollisionEntries =
    std::unordered_map<tesseract_common::LinkNamesPair, std::string, tesseract_common::PairHash>;

bool operator==(const AllowedCollisionEntries& entries_1, const AllowedCollisionEntries& entries_2);

class AllowedCollisionMatrix
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<AllowedCollisionMatrix>;
  using ConstPtr = std::shared_ptr<const AllowedCollisionMatrix>;

  AllowedCollisionMatrix() = default;
  virtual ~AllowedCollisionMatrix() = default;
  AllowedCollisionMatrix(const AllowedCollisionMatrix&) = default;
  AllowedCollisionMatrix& operator=(const AllowedCollisionMatrix&) = default;
  AllowedCollisionMatrix(AllowedCollisionMatrix&&) = default;
  AllowedCollisionMatrix& operator=(AllowedCollisionMatrix&&) = default;

  /**
   * @brief Disable collision between two collision objects
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  /**
   * @brief Get all of the entries in the allowed collision matrix
   * @return AllowedCollisionEntries an unordered map containing all allowed
   *         collision entries. The keys of the unordered map are a std::pair
   *         of the link names in the allowed collision pair.
   */
  const AllowedCollisionEntries& getAllAllowedCollisions() const;

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name);

  /**
   * @brief This checks if two links are allowed to be in collision
   * @param link_name1 First link name
   * @param link_name2 Second link anme
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

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

std::ostream& operator<<(std::ostream& os, const AllowedCollisionMatrix& acm);
}  // namespace tesseract_common

BOOST_CLASS_EXPORT_KEY(tesseract_common::AllowedCollisionMatrix)

#endif  // TESSERACT_SCENE_GRAPH_ALLOWED_COLLISION_MATRIX_H
