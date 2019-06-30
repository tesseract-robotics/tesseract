#ifndef TESSERACT_COMMON_TYPES_H
#define TESSERACT_COMMON_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{

  template <typename T>
  using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

  template <typename Key, typename Value>
  using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

  template <typename Key, typename Value>
  using AlignedUnorderedMap = std::unordered_map<Key,
                                                 Value,
                                                 std::hash<Key>,
                                                 std::equal_to<Key>,
                                                 Eigen::aligned_allocator<std::pair<const Key, Value>>>;

  using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
  using VectorVector4d = AlignedVector<Eigen::Vector4d>;
  using VectorVector3d = std::vector<Eigen::Vector3d>;
  using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;

  using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ;

}
#endif // TESSERACT_COMMON_TYPES_H
