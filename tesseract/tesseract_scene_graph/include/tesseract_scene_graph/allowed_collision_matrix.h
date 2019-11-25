#ifndef TESSERACT_SCENE_GRAPH_ALLOWED_COLLISION_MATRIX_H
#define TESSERACT_SCENE_GRAPH_ALLOWED_COLLISION_MATRIX_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_scene_graph
{
class AllowedCollisionMatrix
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<AllowedCollisionMatrix>;
  using ConstPtr = std::shared_ptr<const AllowedCollisionMatrix>;

  using LinkNamesPair = std::pair<const std::string, const std::string>;
  struct PairHash
  {
    std::size_t operator()(const LinkNamesPair& pair) const
    {
      return std::hash<std::string>()(pair.first + pair.second);
    }
  };
  using AllowedCollisionEntries = std::unordered_map<LinkNamesPair, std::string, PairHash>;

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
                                   const std::string& reason)
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    lookup_table_[link_pair] = reason;
  }

  /**
   * @brief Get all of the entries in the allowed collision matrix
   * @return AllowedCollisionEntries an unordered map containing all allowed
   *         collision entries. The keys of the unordered map are a std::pair
   *         of the link names in the allowed collision pair.
   */
  const AllowedCollisionEntries& getAllAllowedCollisions() const { return lookup_table_; }

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    lookup_table_.erase(link_pair);
  }

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name)
  {
    for (auto it = lookup_table_.begin(); it != lookup_table_.end() /* not hoisted */; /* no increment */)
    {
      if (it->first.first == link_name || it->first.second == link_name)
      {
        it = lookup_table_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  /**
   * @brief This checks if two links are allowed to be in collision
   * @param link_name1 First link name
   * @param link_name2 Second link anme
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    return (lookup_table_.find(link_pair) != lookup_table_.end());
  }

  /**
   * @brief Clears the list of allowed collisions, so that no collision will be
   *        allowed.
   */
  void clearAllowedCollisions() { lookup_table_.clear(); }

private:
  AllowedCollisionEntries lookup_table_;

  /**
   * @brief Create a pair of strings, where the pair.first is always <= pair.second
   * @param link_name1 First link name
   * @param link_name2 Second link anme
   * @return LinkNamesPair a lexicographically sorted pair of strings
   */
  static inline LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
  {
    if (link_name1 <= link_name2)
      return std::make_pair(link_name1, link_name2);

    return std::make_pair(link_name2, link_name1);
  }
};

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_ALLOWED_COLLISION_MATRIX_H
