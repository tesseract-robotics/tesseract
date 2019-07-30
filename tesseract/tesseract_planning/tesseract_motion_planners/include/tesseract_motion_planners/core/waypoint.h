/**
 * @file waypoint_definitions.h
 * @brief The class that defines common types of waypoints that can be sent to the planners
 *
 * @author Matthew Powelson
 * @date February 29, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_WAYPOINT_DEFINITIONS_H
#define TESSERACT_MOTION_PLANNERS_WAYPOINT_DEFINITIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
/** @brief Used to specify the type of waypoint. Corresponds to a derived class of Waypoint*/
enum class WaypointType
{
  JOINT_WAYPOINT,
  JOINT_TOLERANCED_WAYPOINT,
  CARTESIAN_WAYPOINT // All joint waypoint must come before this type and all cartesian must come after
};

inline bool isCartesianWaypointType(WaypointType type)
{
  return (type >= WaypointType::CARTESIAN_WAYPOINT);
}

inline bool isJointWaypointType(WaypointType type)
{
  return (type < WaypointType::CARTESIAN_WAYPOINT);
}

/** @brief Defines a generic way of sending waypoints to a Tesseract Planner */
class Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Waypoint>;
  using ConstPtr = std::shared_ptr<const Waypoint>;

  Waypoint(WaypointType type) : waypoint_type_(type) {}
  virtual ~Waypoint() = default;

  /** @brief Returns the type of waypoint so that it may be cast back to the derived type */
  WaypointType getType() const { return waypoint_type_; }

  /**
   * @brief If false, this value is used as a guide rather than a rigid waypoint (Default=true)
   * Example: In Trajopt, is_critical=true => constraint, is_critical=false => cost
   */
  virtual bool isCritical() const { return is_critical_; }

  /**
   * @brief Set if this waypoint is a critical waypoint.
   *
   * If false, this value is used as a guide rather than a rigid waypoint (Default=true)
   *
   * @param is_critical In Trajopt, is_critical=true => constraint, is_critical=false => cost
   */
  virtual void setIsCritical(bool is_critical) { is_critical_ = is_critical; }

  /**
   * @brief Set coefficients used to weight different terms in the waypoints
   *
   * For example: joint 1 vs joint 2 of the same waypoint or waypoint 1 vs waypoint 2
   * Note: Each planner should define defaults for this when they are not set.
   *
   * @return True if valid, otherwise false
   */
  virtual bool setCoefficients(Eigen::VectorXd coefficients) { coeffs_ = std::move(coefficients); }

  /**
   * @brief Get coefficients used to weight different terms in the waypoints
   *
   * For example: joint 1 vs joint 2 of the same waypoint or waypoint 1 vs waypoint 2
   * Note: Each planner should define defaults for this when they are not set.
   *
   * @return A vector of coefficients
   */
  virtual const Eigen::VectorXd& getCoefficients() const { return coeffs_; }

protected:
  /** @brief Should be set by the derived class for casting Waypoint back to appropriate derived class type */
  WaypointType waypoint_type_;

  /** @brief coefficients used to weight different terms in the waypoints */
  Eigen::VectorXd coeffs_;

  /**
   * @brief If false, this value is used as a guide rather than a rigid waypoint (Default=true)
   * Example: In Trajopt, is_critical=true => constraint, is_critical=false => cost
   */
  bool is_critical_ = true;
};
/** @brief Defines a joint position waypoint for use with Tesseract Planners*/
class JointWaypoint : public Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<JointWaypoint>;
  using ConstPtr = std::shared_ptr<const JointWaypoint>;

  JointWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names)
    : Waypoint(WaypointType::JOINT_WAYPOINT)
    , joint_positions_(std::move(joint_positions))
    , joint_names_(std::move(joint_names))
  {
    assert(joint_positions_.size() == joint_names_.size());
    setCoefficients(Eigen::VectorXd::Ones(joint_positions_.size()));
  }

  JointWaypoint(std::vector<double> joint_positions, std::vector<std::string> joint_names)
    : Waypoint(WaypointType::JOINT_WAYPOINT)
    , joint_names_(std::move(joint_names))
  {
    joint_positions_.resize(joint_positions.size());
    for (long i = 0; i < static_cast<long>(joint_positions.size()); ++i)
      joint_positions_[i] = joint_positions[static_cast<size_t>(i)];

    assert(joint_positions_.size() == joint_names_.size());
    setCoefficients(Eigen::VectorXd::Ones(joint_positions_.size()));
  }

  /**
   * @brief Get the joint positions in radians
   * @return A vector of joint positions
   */
  const Eigen::VectorXd& getPositions() const { return joint_positions_; }

  /**
   * @brief Get the joint names
   * @return A vector of joint names
   */
  const std::vector<std::string>& getNames() const { return joint_names_; }

protected:
  Eigen::VectorXd joint_positions_; /**< @brief Joint position in radians */
  std::vector<std::string> joint_names_; /**< @brief Joint names */
};
/** @brief Defines a cartesian position waypoint for use with Tesseract Planners */
class CartesianWaypoint : public Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartesianWaypoint>;
  using ConstPtr = std::shared_ptr<const CartesianWaypoint>;

  CartesianWaypoint(Eigen::Isometry3d cartesian_position)
    : Waypoint(WaypointType::CARTESIAN_WAYPOINT)
    , cartesian_position_(std::move(cartesian_position))
  {
    setCoefficients(Eigen::VectorXd::Ones(6));
  }

  CartesianWaypoint(Eigen::Vector3d position, Eigen::Quaterniond orientation)
    : Waypoint(WaypointType::CARTESIAN_WAYPOINT)
  {
    cartesian_position_.translation() = position;
    cartesian_position_.linear() = orientation.toRotationMatrix();
    setCoefficients(Eigen::VectorXd::Ones(6));
  }

  /**
   * @brief Get the cartesian transform
   * @return The cartesian transform
   */
  const Eigen::Isometry3d& getTransform() const { return cartesian_position_; }

  /** @brief Convenience function that returns the xyz cartesian position contained in cartesian_position_ */
  Eigen::Vector3d getPosition() { return cartesian_position_.translation(); }
  /**
   * @brief Convenience function that returns the wxyz rotation quarternion contained in cartesian_position
   * @return Quaternion(w, x, y, z)
   */
  Eigen::Vector4d getOrientation()
  {
    Eigen::Quaterniond q(cartesian_position_.rotation());
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  }
protected:
  Eigen::Isometry3d cartesian_position_; /**< @brief Pose of this waypoint */
};

/** @brief Defines a joint toleranced position waypoint for use with Tesseract Planners*/
class JointTolerancedWaypoint : public Waypoint
{
public:
  using Ptr = std::shared_ptr<JointTolerancedWaypoint>;
  using ConstPtr = std::shared_ptr<const JointTolerancedWaypoint>;

  JointTolerancedWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names)
    : Waypoint(WaypointType::JOINT_TOLERANCED_WAYPOINT)
    , joint_positions_(std::move(joint_positions))
    , joint_names_(std::move(joint_names))
  {
    assert(joint_positions_.size() == joint_names_.size());
    setCoefficients(Eigen::VectorXd::Ones(joint_positions_.size()));
    setUpperTolerance(Eigen::VectorXd::Zero(joint_positions_.size()));
    setLowerTolerance(Eigen::VectorXd::Zero(joint_positions_.size()));
  }

  JointTolerancedWaypoint(std::vector<double> joint_positions, std::vector<std::string> joint_names)
    : Waypoint(WaypointType::JOINT_TOLERANCED_WAYPOINT)
    , joint_names_(std::move(joint_names))
  {
    joint_positions_.resize(joint_positions.size());
    for (long i = 0; i < static_cast<long>(joint_positions.size()); ++i)
      joint_positions_[i] = joint_positions[static_cast<size_t>(i)];

    assert(joint_positions_.size() == joint_names_.size());
    setCoefficients(Eigen::VectorXd::Ones(joint_positions_.size()));
    setUpperTolerance(Eigen::VectorXd::Zero(joint_positions_.size()));
    setLowerTolerance(Eigen::VectorXd::Zero(joint_positions_.size()));
  }

  /**
   * @brief Get the joint positions in radians
   * @return A vector of joint positions
   */
  const Eigen::VectorXd& getPositions() const { return joint_positions_; }

  /**
   * @brief Get the joint names
   * @return A vector of joint names
   */
  const std::vector<std::string>& getNames() const { return joint_names_; }

  /**
   * @brief Set Amount over joint_positions_ that is allowed (positive radians).
   *
   * The allowed range is joint_positions-lower_tolerance_ to joint_positions_+upper_tolerance
   *
   * @param upper_tolerance The upper tolerance in radians.
   * @return True if valid, otherwise false
   */
  bool setUpperTolerance(Eigen::VectorXd upper_tolerance)
  {
    if (upper_tolerance.size() != joint_positions_.size())
      return false;

    upper_tolerance_ = std::move(upper_tolerance);
    return true;
  }

  /**
   * @brief Get the upper tolerance in radians
   * @return The upper tolerance
   */
  const Eigen::VectorXd& getUpperTolerance() const { return upper_tolerance_; }

  /**
   * @brief Set Amount under joint_positions_ that is allowed (positive radians).
   *
   * The allowed range is joint_positions-lower_tolerance_ to joint_positions_+upper_tolerance
   *
   * @param lower_tolerance The lower tolerance in radians.
   * @return True if valid, otherwise false
   */
  bool setLowerTolerance(Eigen::VectorXd lower_tolerance)
  {
    if (lower_tolerance.size() != joint_positions_.size())
      return false;

    lower_tolerance_ = std::move(lower_tolerance);
    return true;
  }

  /**
   * @brief Get the lower tolerance in radians
   * @return The lower tolerance
   */
  const Eigen::VectorXd& getLowerTolerance() const { return lower_tolerance_; }

protected:
  Eigen::VectorXd joint_positions_; /**< @brief Joint position in radians */
  std::vector<std::string> joint_names_; /**< @brief Joint names */

  /** @brief Amount over joint_positions_ that is allowed (positive radians). */
  Eigen::VectorXd upper_tolerance_;

  /** @brief Amount under joint_positions_ that is allowed (negative radians). */
  Eigen::VectorXd lower_tolerance_;
};

}  // namespace tesseract_motion_planners
#endif
