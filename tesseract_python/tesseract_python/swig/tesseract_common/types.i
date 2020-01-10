/**
 * @file types.i
 * @brief SWIG interface file for tesseract_common/types.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_common/types.h>
%}

%define tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define tesseract_aligned_map_of_aligned_vector(name,Key,Value)
tesseract_aligned_map(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

%define tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
//tesseract_aligned_vector(VectorVector3d, Eigen::Vector3d);
//%template(VectorVector3d) std::vector<Eigen::Vector3d>; // This is handled by eigen.i
tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);

tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);

namespace tesseract_common
{
/*
typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
//TODO: Whis is this not aligned??
typedef std::vector<Eigen::Vector3d> VectorVector3d;
typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> TransformMap;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
*/

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

using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct JointTrajectory
{
  std::vector<std::string> joint_names;   /**< @brief The joint names */
  tesseract_common::TrajArray trajectory; /**< @brief The generated trajectory */
};

}