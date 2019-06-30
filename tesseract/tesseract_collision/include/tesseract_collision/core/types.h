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
  typedef std::vector<tesseract_geometry::GeometryConstPtr> CollisionShapesConst;
  typedef tesseract_geometry::GeometryConstPtr CollisionShapeConstPtr;
  typedef tesseract_geometry::GeometryPtr CollisionShapePtr;

  /**
   * @brief Should return true if contact allowed, otherwise false.
   *
   * Also the order of strings should not matter, the function should handled by the function.
   */
  typedef std::function<bool(const std::string&, const std::string&)> IsContactAllowedFn;

  namespace ContinouseCollisionTypes
  {
  enum ContinouseCollisionType
  {
    CCType_None,
    CCType_Time0,
    CCType_Time1,
    CCType_Between
  };
  }
  typedef ContinouseCollisionTypes::ContinouseCollisionType ContinouseCollisionType;

  namespace ContactTestTypes
  {
  enum ContactTestType
  {
    FIRST = 0,   /**< Return at first contact for any pair of objects */
    CLOSEST = 1, /**< Return the global minimum for a pair of objects */
    ALL = 2,     /**< Return all contacts for a pair of objects */
    LIMITED = 3  /**< Return limited set of contacts for a pair of objects */
  };
  }
  typedef ContactTestTypes::ContactTestType ContactTestType;

  struct ContactResult
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double distance;
    int type_id[2];
    std::string link_names[2];
    Eigen::Vector3d nearest_points[2];
    Eigen::Vector3d normal;
    Eigen::Vector3d cc_nearest_points[2];
    double cc_time;
    ContinouseCollisionType cc_type;

    ContactResult() { clear(); }
    /// Clear structure data
    void clear()
    {
      distance = std::numeric_limits<double>::max();
      nearest_points[0].setZero();
      nearest_points[1].setZero();
      link_names[0] = "";
      link_names[1] = "";
      type_id[0] = 0;
      type_id[1] = 0;
      normal.setZero();
      cc_nearest_points[0].setZero();
      cc_nearest_points[1].setZero();
      cc_time = -1;
      cc_type = ContinouseCollisionType::CCType_None;
    }
  };

  using ContactResultVector = tesseract_common::AlignedVector<ContactResult> ;
  using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;

  inline std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v)
  {
      v.clear();
      size_t l = 0;
      for (const auto& mv : m)
        l+=mv.second.size();

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
}

#endif // TESSERACT_COLLISION_TYPES_H
