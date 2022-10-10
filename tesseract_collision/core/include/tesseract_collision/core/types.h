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
#include <array>
#include <unordered_map>
#include <functional>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/types.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/allowed_collision_matrix.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{
using CollisionShapesConst = std::vector<tesseract_geometry::Geometry::ConstPtr>;
using CollisionShapeConstPtr = tesseract_geometry::Geometry::ConstPtr;
using CollisionShapePtr = tesseract_geometry::Geometry::Ptr;
using CollisionMarginData = tesseract_common::CollisionMarginData;
using CollisionMarginOverrideType = tesseract_common::CollisionMarginOverrideType;
using PairsCollisionMarginData = tesseract_common::PairsCollisionMarginData;

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

static const std::vector<std::string> ContactTestTypeStrings = {
  "FIRST",
  "CLOSEST",
  "ALL",
  "LIMITED",
};

struct ContactResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief The distance between two links */
  double distance{ std::numeric_limits<double>::max() };
  /** @brief A user defined type id that is added to the contact shapes */
  std::array<int, 2> type_id{ 0, 0 };
  /** @brief The two links that are in contact */
  std::array<std::string, 2> link_names;
  /** @brief The two shapes that are in contact. Each link can be made up of multiple shapes */
  std::array<int, 2> shape_id{ -1, -1 };
  /** @brief Some shapes like octomap and mesh have subshape (boxes and triangles) */
  std::array<int, 2> subshape_id{ -1, -1 };
  /** @brief The nearest point on both links in world coordinates */
  std::array<Eigen::Vector3d, 2> nearest_points{ Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  /** @brief The nearest point on both links in local(link) coordinates */
  std::array<Eigen::Vector3d, 2> nearest_points_local{ Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  /** @brief The transform of link in world coordinates */
  std::array<Eigen::Isometry3d, 2> transform{ Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity() };
  /**
   * @brief The normal vector to move the two objects out of contact in world coordinates
   *
   * @note This points from link_name[0] to link_name[1], so it shows the direction to move link_name[1] to avoid or get
   *       out of collision with link_name[0].
   */
  Eigen::Vector3d normal{ Eigen::Vector3d::Zero() };
  /** @brief This is between 0 and 1 indicating the point of contact */
  std::array<double, 2> cc_time{ -1, -1 };
  /** @brief The type of continuous contact */
  std::array<ContinuousCollisionType, 2> cc_type{ ContinuousCollisionType::CCType_None,
                                                  ContinuousCollisionType::CCType_None };
  /** @brief The transform of link in world coordinates at its desired final location.
   * Note: This is not the location of the link at the point of contact but the final location the link when performing
   *       continuous collision checking. If you desire the location of contact use cc_time and interpolate between
   *       transform and cc_transform;
   */
  std::array<Eigen::Isometry3d, 2> cc_transform{ Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity() };

  /** @brief Some collision checkers only provide a single contact point for a given pair. This is used to indicate
   * if only one contact point is provided which means nearest_points[0] must equal nearest_points[1].
   */
  bool single_contact_point{ false };

  ContactResult() = default;

  /** @brief reset to default values */
  void clear();
};

using ContactResultVector = tesseract_common::AlignedVector<ContactResult>;
using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;

/**
 * @brief Should return true if contact results are valid, otherwise false.
 *
 * This is used so users may provide a callback to reject/approve collision results in various algorithms.
 */
using IsContactResultValidFn = std::function<bool(const ContactResult&)>;

/** @brief The ContactRequest struct */
struct ContactRequest
{
  /** @brief This controls the exit condition for the contact test type */
  ContactTestType type = ContactTestType::ALL;

  /** @brief This enables the calculation of penetration contact data if two objects are in collision */
  bool calculate_penetration = true;

  /** @brief This enables the calculation of distance data if two objects are within the contact threshold */
  bool calculate_distance = true;

  /** @brief This is used if the ContactTestType is set to LIMITED, where the test will exit when number of contacts
   * reach this limit */
  long contact_limit = 0;

  /** @brief This provides a user defined function approve/reject contact results */
  IsContactResultValidFn is_valid = nullptr;

  ContactRequest(ContactTestType type = ContactTestType::ALL);
};

std::size_t flattenMoveResults(ContactResultMap&& m, ContactResultVector& v);

std::size_t flattenCopyResults(const ContactResultMap& m, ContactResultVector& v);

/**
 * @brief This data is intended only to be used internal to the collision checkers as a container and should not
 *        be externally used by other libraries or packages.
 */
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData() = default;
  ContactTestData(const std::vector<std::string>& active,
                  CollisionMarginData collision_margin_data,
                  IsContactAllowedFn fn,
                  ContactRequest req,
                  ContactResultMap& res);

  /** @brief A vector of active links */
  const std::vector<std::string>* active = nullptr;

  /** @brief The current contact_distance threshold */
  CollisionMarginData collision_margin_data{ 0 };

  /** @brief The allowed collision function used to check if two links should be excluded from collision checking */
  IsContactAllowedFn fn = nullptr;

  /** @brief The type of contact request data */
  ContactRequest req;

  /** @brief Distance query results information */
  ContactResultMap* res = nullptr;

  /** @brief Indicate if search is finished */
  bool done = false;
};

/**
 * @brief High level descriptor used in planners and utilities to specify what kind of collision check is desired.
 *
 * DISCRETE - Discrete contact manager using only steps specified
 * LVS_DISCRETE - Discrete contact manager interpolating using longest valid segment
 * CONTINUOUS - Continuous contact manager using only steps specified
 * LVS_CONTINUOUS - Continuous contact manager interpolating using longest valid segment
 */
enum class CollisionEvaluatorType
{
  /** @brief None */
  NONE,
  /** @brief Discrete contact manager using only steps specified */
  DISCRETE,
  /** @brief Discrete contact manager interpolating using longest valid segment */
  LVS_DISCRETE,
  /** @brief Continuous contact manager using only steps specified */
  CONTINUOUS,
  /** @brief Continuous contact manager interpolating using longest valid segment */
  LVS_CONTINUOUS
};

/** @brief Identifies how the provided AllowedCollisionMatrix should be applied relative to the isAllowedFn in the
 * contact manager */
enum class ACMOverrideType
{
  /** @brief Do not apply AllowedCollisionMatrix */
  NONE,
  /** @brief Replace the current IsContactAllowedFn with one generated from the ACM provided */
  ASSIGN,
  /** @brief New IsContactAllowedFn combines the contact manager fn and the ACM generated fn with and AND */
  AND,
  /** @brief New IsContactAllowedFn combines the contact manager fn and the ACM generated fn with and OR */
  OR,
};

/**
 * @brief Contains parameters used to configure a contact manager before a series of contact checks.
 *
 * It should not contain information that is usually specific to a single contactTest such as CollisionObjectTransforms
 * or specific to the way contactTests are carried out such as LVS parameters
 *
 * @note Active links were not added to this config since this config could be shared by multiple manipulators, and
 * those are set based on which one is being checked
 */
struct ContactManagerConfig
{
  ContactManagerConfig() = default;
  ContactManagerConfig(double default_margin);

  /** @brief Identify how the collision margin data should be applied to the contact manager */
  CollisionMarginOverrideType margin_data_override_type{ CollisionMarginOverrideType::NONE };
  /** @brief Stores information about how the margins allowed between collision objects*/
  CollisionMarginData margin_data;

  /** @brief Additional AllowedCollisionMatrix to consider for this collision check.  */
  tesseract_common::AllowedCollisionMatrix acm;
  /** @brief Specifies how to combine the IsContactAllowedFn from acm with the one preset in the contact manager */
  ACMOverrideType acm_override_type{ ACMOverrideType::OR };

  /** @brief Each key is an object name. Objects will be enabled/disabled based on the value. Objects that aren't in the
   * map are unmodified from the defaults*/
  std::unordered_map<std::string, bool> modify_object_enabled;
};

/**
 * @brief This is a high level structure containing common information that collision checking utilities need. The goal
 * of this config is to allow all collision checking utilities and planners to use the same data structure
 */
struct CollisionCheckConfig
{
  CollisionCheckConfig() = default;
  CollisionCheckConfig(double default_margin,
                       ContactRequest request = ContactRequest(),
                       CollisionEvaluatorType type = CollisionEvaluatorType::DISCRETE,
                       double longest_valid_segment_length = 0.005);

  /** @brief Used to configure the contact manager prior to a series of checks */
  ContactManagerConfig contact_manager_config;

  /** @brief ContactRequest that will be used for this check. Default test type: FIRST*/
  ContactRequest contact_request;
  /** @brief Specifies the type of collision check to be performed. Default: DISCRETE */
  CollisionEvaluatorType type{ CollisionEvaluatorType::DISCRETE };
  /** @brief Longest valid segment to use if type supports lvs. Default: 0.005*/
  double longest_valid_segment_length{ 0.005 };
};
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_TYPES_H
