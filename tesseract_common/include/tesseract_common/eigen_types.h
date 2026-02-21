#ifndef TESSERACT_COMMON_EIGEN_UTILS_H
#define TESSERACT_COMMON_EIGEN_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <map>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::common
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
using VectorVector2d = AlignedVector<Eigen::Vector2d>;
using TransformMap = AlignedUnorderedMap<std::string, Eigen::Isometry3d>;
using Toolpath = AlignedVector<VectorIsometry3d>;
using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
}  // namespace tesseract::common

#endif  // TESSERACT_COLLISION_EIGEN_UTILS_H
