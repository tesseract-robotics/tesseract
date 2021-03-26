/**
 * @file types.h
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <boost/filesystem.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/kinematic_limits.h>

namespace tesseract_common
{
/** @brief Enable easy switching to std::filesystem when available */
namespace fs = boost::filesystem;

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
using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;
using Toolpath = AlignedVector<VectorIsometry3d>;

using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using LinkNamesPair = std::pair<std::string, std::string>;
struct PairHash
{
  std::size_t operator()(const LinkNamesPair& pair) const { return std::hash<std::string>()(pair.first + pair.second); }
};
/**
 * @brief Create a pair of strings, where the pair.first is always <= pair.second.
 *
 * This is commonly used along with PairHash as the key to an unordered_map<LinkNamesPair, Type, PairHash>
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

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_TYPES_H
