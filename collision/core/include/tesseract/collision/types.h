/**
 * @file types.h
 * @brief Tesseracts Collision Common Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <array>
#include <unordered_map>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <tesseract/common/any_poly.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/geometry/fwd.h>

namespace tesseract::collision
{
using CollisionShapeConstPtr = std::shared_ptr<const tesseract::geometry::Geometry>;
using CollisionShapePtr = std::shared_ptr<tesseract::geometry::Geometry>;
using CollisionShapesConst = std::vector<CollisionShapeConstPtr>;
using CollisionMarginData = tesseract::common::CollisionMarginData;
using CollisionMarginPairData = tesseract::common::CollisionMarginPairData;
using CollisionMarginPairOverrideType = tesseract::common::CollisionMarginPairOverrideType;

class ContactResultValidator;

enum class ContinuousCollisionType : std::uint8_t
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

enum class ContactTestType : std::uint8_t
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
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

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

  bool operator==(const ContactResult& rhs) const;
  bool operator!=(const ContactResult& rhs) const;
};

using ContactResultVector = tesseract::common::AlignedVector<ContactResult>;

/**
 * @brief This structure hold contact results for link pairs
 * @details A custom class was implemented to avoid a large number of heap allocations during motion which avoids full
 * clearing the map. This class provides methods const container methods for access the internal map and has
 * two distinct different when it comes to the clear, size and release methods.
 *
 * The clear method does not call clear on the map but instead it loops over all entries and calls clear on
 * the vector being stored. This allows the memory to remain with the map and not get release for each of the vectors
 * stored in the map.
 *
 * The size method loops over the map and counts those that have vectors which are not empty.
 *
 * The release method actually calls clear on the internal map relasing all memory.
 *
 * @todo This should be updated to leverage a object pool for `ContactResultVector` where in the set and add methods
 * it would check it the pair exists and if not it would pull from the object pool.
 */
class ContactResultMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using KeyType = std::pair<std::string, std::string>;
  using MappedType = ContactResultVector;
  using ContainerType = tesseract::common::AlignedMap<KeyType, MappedType>;
  using ConstReferenceType = typename tesseract::common::AlignedMap<KeyType, MappedType>::const_reference;
  using ConstIteratorType = typename tesseract::common::AlignedMap<KeyType, MappedType>::const_iterator;
  using PairType = typename std::pair<const KeyType, MappedType>;
  using FilterFn = std::function<void(PairType&)>;

  /**
   * @brief Add contact results for the provided key
   * @param key The key to append the results to
   * @param result The results to add
   */
  ContactResult& addContactResult(const KeyType& key, ContactResult result);

  /**
   * @brief Add contact results for the provided key
   * @param key The key to append the results to
   * @param result The results to add
   */
  ContactResult& addContactResult(const KeyType& key, const MappedType& results);

  /**
   * @brief Set contact results for the provided key
   * @param key The key to assign the provided results to
   * @param result The results to assign
   */
  ContactResult& setContactResult(const KeyType& key, ContactResult result);

  /**
   * @brief Set contact results for the provided key
   * @param key The key to assign the provided results to
   * @param result The results to assign
   */
  ContactResult& setContactResult(const KeyType& key, const MappedType& results);

  /**
   * @brief This processes interpolated contact results by updating the cc_time and cc_type and then adds the result
   * @details This is copied from the trajopt utility processInterpolatedCollisionResults
   * @param sub_segment_results The interpolated results to process
   * @param sub_segment_index The current sub segment index
   * @param sub_segment_last_index The last sub segment index
   * @param active_link_names The active link names
   * @param segment_dt The segment dt
   * @param discrete If discrete contact checker was used
   * @param filter An option filter to exclude results
   */
  void addInterpolatedCollisionResults(ContactResultMap& sub_segment_results,
                                       long sub_segment_index,
                                       long sub_segment_last_index,
                                       const std::vector<std::string>& active_link_names,
                                       double segment_dt,
                                       bool discrete,
                                       const ContactResultMap::FilterFn& filter = nullptr);

  // Flatten functions
  void flattenMoveResults(ContactResultVector& v);
  void flattenCopyResults(ContactResultVector& v) const;
  void flattenWrapperResults(std::vector<std::reference_wrapper<ContactResult>>& v);
  void flattenWrapperResults(std::vector<std::reference_wrapper<const ContactResult>>& v) const;

  /**
   * @brief Filter out results using the provided function
   * @param fn The filter function
   */
  void filter(const FilterFn& filter);

  /**
   * @brief Get the total number of contact results storted
   * @return The number of contact results
   */
  long count() const;

  /**
   * @brief Get the size of the map
   * @details This loops over the internal map and counts entries with contacts
   * @return The number of entries with contacts
   */
  std::size_t size() const;

  /**
   * @brief Check if results are present
   * @return
   */
  bool empty() const;

  /**
   * @brief This is a consurvative clear.
   * @details This does not call clear on the internal map but instead loops over each link pair entry and calls clear
   * on the underlying vector. This way the vector capacity remains the same to avoid uneccessary heap allocation for
   * subsequent contact requests.
   * @note Use release to fully clear the internal data structure
   */
  void clear();

  /** @brief Remove map entries with no contact results */
  void shrinkToFit();

  /** @brief Fully clear all internal data */
  void release();

  /**
   * @brief Get the underlying container
   * @warning Do not use this for anything other than debugging or serialization
   */
  const ContainerType& getContainer() const;

  ///////////////
  // Iterators //
  ///////////////
  /** @brief returns an iterator to the beginning */
  ConstIteratorType begin() const;
  /** @brief returns an iterator to the end */
  ConstIteratorType end() const;
  /** @brief returns an iterator to the beginning */
  ConstIteratorType cbegin() const;
  /** @brief returns an iterator to the end */
  ConstIteratorType cend() const;

  ////////////////////
  // Element Access //
  ////////////////////
  /** @brief access specified element with bounds checking */
  const ContactResultVector& at(const KeyType& key) const;
  ConstIteratorType find(const KeyType& key) const;

  bool operator==(const ContactResultMap& rhs) const;
  bool operator!=(const ContactResultMap& rhs) const;

  /**
   * @brief Get a brief summary of the most frequently colliding link pair
   * @return A string containing the collision summary
   */
  std::string getSummary() const;

private:
  ContainerType data_;
  long count_{ 0 };
};

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
  std::shared_ptr<const ContactResultValidator> is_valid;

  ContactRequest(ContactTestType type = ContactTestType::ALL);

  bool operator==(const ContactRequest& rhs) const;
  bool operator!=(const ContactRequest& rhs) const;
};

/**
 * @brief This data is intended only to be used internal to the collision checkers as a container and should not
 *        be externally used by other libraries or packages.
 */
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData() = default;
  ContactTestData(CollisionMarginData collision_margin_data,
                  std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator,
                  ContactRequest req,
                  ContactResultMap& res);

  /** @brief The current contact_distance threshold */
  CollisionMarginData collision_margin_data{ 0 };

  /** @brief The allowed collision function used to check if two links should be excluded from collision checking */
  std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator;

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
enum class CollisionEvaluatorType : std::uint8_t
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

/** @brief The mode used to check program */
enum class CollisionCheckProgramType : std::uint8_t
{
  /** @brief Check all states */
  ALL,
  /** @brief Check all states except the start state */
  ALL_EXCEPT_START,
  /** @brief Check all states except the end state */
  ALL_EXCEPT_END,
  /** @brief Check only the start state */
  START_ONLY,
  /** @brief Check only the end state */
  END_ONLY,
  /** @brief Check only the intermediate states */
  INTERMEDIATE_ONLY
};

/** @brief Controls when collision checking exits or continues collecting results */
enum class CollisionCheckExitType : std::uint8_t
{
  /** @brief Exit as soon as any collision is detected */
  FIRST,
  /** @brief For LVS, stop at first collision per step; continue to next step */
  ONE_PER_STEP,
  /** @brief Collect all collisions */
  ALL
};

/** @brief Identifies how the provided AllowedCollisionMatrix should be applied relative to the isAllowedFn in the
 * contact manager */
enum class ACMOverrideType : std::uint8_t
{
  /** @brief Do not apply AllowedCollisionMatrix */
  NONE,
  /** @brief Replace the current ContactAllowedValidator with one generated from the ACM provided */
  ASSIGN,
  /** @brief New ContactAllowedValidator combines the contact manager fn and the ACM generated fn with and AND */
  AND,
  /** @brief New ContactAllowedValidator combines the contact manager fn and the ACM generated fn with and OR */
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

  /** @brief override the default contact margin */
  std::optional<double> default_margin;

  /** @brief Identify how the collision margin pair data should be applied to the contact manager */
  CollisionMarginPairOverrideType pair_margin_override_type{ CollisionMarginPairOverrideType::NONE };
  /** @brief Stores collision margin pair data */
  CollisionMarginPairData pair_margin_data;

  /** @brief Additional AllowedCollisionMatrix to consider for this collision check.  */
  tesseract::common::AllowedCollisionMatrix acm;
  /** @brief Specifies how to combine the ContactAllowedValidator from acm with the one preset in the contact manager */
  ACMOverrideType acm_override_type{ ACMOverrideType::NONE };

  /** @brief Each key is an object name. Objects will be enabled/disabled based on the value. Objects that aren't in the
   * map are unmodified from the defaults*/
  std::unordered_map<std::string, bool> modify_object_enabled;

  /**
   * @brief Increment all margins by input amount. Useful for inflating or reducing margins
   * @param increment Amount to increment margins
   */
  void incrementMargins(double increment);

  /**
   * @brief Scale all margins by input value
   * @param scale Value by which all margins are multiplied
   */
  void scaleMargins(double scale);

  /**
   * @brief Check for errors and throw exception if they exist
   * @throws an exception if `margin_data_override_type` is set to `NONE`, but margin pair data exist; or if
   * `acm_override_type` is set to `NONE` but allowed collision entries exist.
   */
  void validate() const;

  bool operator==(const ContactManagerConfig& rhs) const;
  bool operator!=(const ContactManagerConfig& rhs) const;
};

/**
 * @brief This is a high level structure containing common information that collision checking utilities need. The goal
 * of this config is to allow all collision checking utilities and planners to use the same data structure
 */
struct CollisionCheckConfig
{
  CollisionCheckConfig(ContactRequest request = ContactRequest(),
                       CollisionEvaluatorType type = CollisionEvaluatorType::DISCRETE,
                       double longest_valid_segment_length = 0.005,
                       CollisionCheckProgramType check_program_mode = CollisionCheckProgramType::ALL,
                       CollisionCheckExitType exit_condition = CollisionCheckExitType::FIRST);

  /** @brief ContactRequest that will be used for this check. Default test type: ALL*/
  ContactRequest contact_request;

  /** @brief Specifies the type of collision check to be performed. Default: DISCRETE */
  CollisionEvaluatorType type{ CollisionEvaluatorType::DISCRETE };

  /** @brief Longest valid segment to use if type supports lvs. Default: 0.005*/
  double longest_valid_segment_length{ 0.005 };

  /** @brief Secifies the mode used when collision checking program/trajectory. Default: ALL */
  CollisionCheckProgramType check_program_mode{ CollisionCheckProgramType::ALL };

  /** @brief The condition in which the trajectory collision check will exit. Default: FIRST */
  CollisionCheckExitType exit_condition{ CollisionCheckExitType::FIRST };

  bool operator==(const CollisionCheckConfig& rhs) const;
  bool operator!=(const CollisionCheckConfig& rhs) const;
};

/**
 * @brief The ContactTrajectorySubstepResults struct is the lowest level struct for tracking contacts in a trajectory.
 * This struct is used for substeps between waypoints in a trajectory when a longest valid segment is used, storing the
 * relevant states of the substep.
 */
struct ContactTrajectorySubstepResults
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ContactTrajectorySubstepResults() = default;
  ContactTrajectorySubstepResults(int substep, const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state);
  ContactTrajectorySubstepResults(int substep, const Eigen::VectorXd& state);

  using UPtr = std::unique_ptr<ContactTrajectorySubstepResults>;

  operator bool() const;

  void addContact(int substep_number,
                  const Eigen::VectorXd& start_substate,
                  const Eigen::VectorXd& end_substate,
                  const tesseract::collision::ContactResultMap& new_contacts);

  int numContacts() const;

  tesseract::collision::ContactResultVector worstCollision() const;

  tesseract::collision::ContactResultMap contacts;
  int substep = -1;
  Eigen::VectorXd state0;
  Eigen::VectorXd state1;
};

/**
 * @brief The ContactTrajectoryStepResults struct is the second level struct for tracking contacts in a trajectory. This
 * struct stores all the substep contact information as well as the start and end state of the given step in the
 * trajectory.
 */
struct ContactTrajectoryStepResults
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ContactTrajectoryStepResults() = default;
  ContactTrajectoryStepResults(int step_number,
                               const Eigen::VectorXd& start_state,
                               const Eigen::VectorXd& end_state,
                               int num_substeps);
  ContactTrajectoryStepResults(int step_number, const Eigen::VectorXd& state);

  using UPtr = std::unique_ptr<ContactTrajectoryStepResults>;

  operator bool() const;

  void addContact(int step_number,
                  int substep_number,
                  int num_substeps,
                  const Eigen::VectorXd& start_state,
                  const Eigen::VectorXd& end_state,
                  const Eigen::VectorXd& start_substate,
                  const Eigen::VectorXd& end_substate,
                  const tesseract::collision::ContactResultMap& contacts);

  void resize(int num_substeps);

  int numSubsteps() const;

  int numContacts() const;

  ContactTrajectorySubstepResults worstSubstep() const;

  tesseract::collision::ContactResultVector worstCollision() const;

  ContactTrajectorySubstepResults mostCollisionsSubstep() const;

  std::vector<ContactTrajectorySubstepResults> substeps;
  int step = -1;
  Eigen::VectorXd state0;
  Eigen::VectorXd state1;
  int total_substeps = 0;
};

/**
 * @brief The ContactTrajectoryResults struct is the top level struct for tracking contacts in a trajectory. This struct
 * stores all the steps and therefore all the contacts in a trajectory. It also exposes a method for returning a contact
 * summary table as a string for printing to a terminal.
 */
struct ContactTrajectoryResults
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  ContactTrajectoryResults() = default;
  ContactTrajectoryResults(std::vector<std::string> j_names);
  ContactTrajectoryResults(std::vector<std::string> j_names, int num_steps);

  using UPtr = std::unique_ptr<ContactTrajectoryResults>;

  operator bool() const;

  void addContact(int step_number,
                  int substep_number,
                  int num_substeps,
                  const Eigen::VectorXd& start_state,
                  const Eigen::VectorXd& end_state,
                  const Eigen::VectorXd& start_substate,
                  const Eigen::VectorXd& end_substate,
                  const tesseract::collision::ContactResultMap& contacts);

  void resize(int num_steps);

  int numSteps() const;

  int numContacts() const;

  ContactTrajectoryStepResults worstStep() const;

  tesseract::collision::ContactResultVector worstCollision() const;

  ContactTrajectoryStepResults mostCollisionsStep() const;

  std::stringstream trajectoryCollisionResultsTable() const;

  std::stringstream collisionFrequencyPerLink() const;

  std::stringstream condensedSummary() const;

  std::vector<ContactTrajectoryStepResults> steps;
  std::vector<std::string> joint_names;
  int total_steps = 0;
};

using ContactResultAnyPoly = tesseract::common::AnyWrapper<ContactResult>;
using ContactResultMapAnyPoly = tesseract::common::AnyWrapper<ContactResultMap>;
using ContactResultMapVectorAnyPoly =
    tesseract::common::AnyWrapper<std::vector<tesseract::collision::ContactResultMap>>;

}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_TYPES_H
