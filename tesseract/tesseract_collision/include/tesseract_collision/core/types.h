/**
 * @file types.h
 * @brief Tesseracts Collision Common Types
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
#ifndef TESSERACT_COLLISION_TYPES_H
#define TESSERACT_COLLISION_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <functional>
#include <boost/bind.hpp>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{
using CollisionShapesConst = std::vector<tesseract_geometry::Geometry::ConstPtr>;
using CollisionShapeConstPtr = tesseract_geometry::Geometry::ConstPtr;
using CollisionShapePtr = tesseract_geometry::Geometry::Ptr;

/**
 * @brief Should return true if contact allowed, otherwise false.
 *
 * Also the order of strings should not matter, the function should handled by the function.
 */
using IsContactAllowedFn = std::function<bool(const std::string&, const std::string&)>;

enum class ContinuousCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

enum class ContactTestType
{
  FIRST = 0,   /**< Return at first contact for any pair of objects */
  CLOSEST = 1, /**< Return the global minimum for a pair of objects */
  ALL = 2,     /**< Return all contacts for a pair of objects */
  LIMITED = 3  /**< Return limited set of contacts for a pair of objects */
};

struct ContactResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief The distance between two links */
  double distance;
  /** @brief A user defined type id that is added to the contact shapes */
  int type_id[2];
  /** @brief The two links that are in contact */
  std::string link_names[2];
  /** @brief The two shapes that are in contact. Each link can be made up of multiple shapes */
  int shape_id[2];
  /** @brief Some shapes like octomap and mesh have subshape (boxes and triangles) */
  int subshape_id[2];
  /** @brief The nearest point on both links in world coordinates */
  Eigen::Vector3d nearest_points[2];
  /** @brief The nearest point on both links in local(link) coordinates */
  Eigen::Vector3d nearest_points_local[2];
  /** @brief The transform of link in world coordinates */
  Eigen::Isometry3d transform[2];
  /** @brief The normal vector to move the two objects out of contact in world coordinates */
  Eigen::Vector3d normal;
  /** @brief This is between 0 and 1 indicating the point of contact */
  double cc_time[2];
  /** @brief The type of continuous contact */
  std::array<ContinuousCollisionType, 2> cc_type;
  /** @brief The transform of link in world coordinates at its desired final location.
   * Note: This is not the location of the link at the point of contact but the final location the link when performing
   *       continuous collision checking. If you desire the location of contact use cc_time and interpolate between
   *       transform and cc_transform;
   */
  Eigen::Isometry3d cc_transform[2];

  ContactResult() { clear(); }

  /** @brief reset to default values */
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    nearest_points_local[0].setZero();
    nearest_points_local[1].setZero();
    transform[0] = Eigen::Isometry3d::Identity();
    transform[1] = Eigen::Isometry3d::Identity();
    link_names[0] = "";
    link_names[1] = "";
    shape_id[0] = -1;
    shape_id[1] = -1;
    subshape_id[0] = -1;
    subshape_id[1] = -1;
    type_id[0] = 0;
    type_id[1] = 0;
    normal.setZero();
    cc_time[0] = -1;
    cc_time[1] = -1;
    cc_type[0] = ContinuousCollisionType::CCType_None;
    cc_type[1] = ContinuousCollisionType::CCType_None;
    cc_transform[0] = Eigen::Isometry3d::Identity();
    cc_transform[1] = Eigen::Isometry3d::Identity();
  }
};

using ContactResultVector = tesseract_common::AlignedVector<ContactResult>;
using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;

inline std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v)
{
  v.clear();
  size_t l = 0;
  for (const auto& mv : m)
    l += mv.second.size();

  v.reserve(l);
  for (const auto& mv : m)
    std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

/// Contact test data and query results information
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData(const std::vector<std::string>& active,
                  const double& contact_distance,
                  const IsContactAllowedFn& fn,
                  const ContactTestType& type,
                  ContactResultMap& res)
    : active(active), contact_distance(contact_distance), fn(fn), type(type), res(res), done(false)
  {
  }

  const std::vector<std::string>& active;
  const double& contact_distance;
  const IsContactAllowedFn& fn;
  const ContactTestType& type;

  /// Destance query results information
  ContactResultMap& res;

  /// Indicate if search is finished
  bool done;
};
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_TYPES_H
