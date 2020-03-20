/**
 * @file forward_kinematics.h
 * @brief Forward kinematics functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H
#define TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Geometry>
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/macros.h>

// Example Reference: https://foonathan.net/2020/01/type-erasure/
namespace tesseract_kinematics
{

namespace detail
{
/**
 * @brief This defines the Forward Kinematics Interface. Those who wish to develope a forward kinematics that works
 *        within Tesseract it must have these public function.
 */
struct ForwardKinematicsInnerBase
{
  ForwardKinematicsInnerBase() = default;
  virtual ~ForwardKinematicsInnerBase() = default;
  ForwardKinematicsInnerBase(const ForwardKinematicsInnerBase&) = delete;
  ForwardKinematicsInnerBase& operator=(const ForwardKinematicsInnerBase&) = delete;
  ForwardKinematicsInnerBase(ForwardKinematicsInnerBase&&) = delete;
  ForwardKinematicsInnerBase& operator=(ForwardKinematicsInnerBase&&) = delete;

  virtual bool calcFwdKin(Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  virtual bool calcFwdKin(tesseract_common::VectorIsometry3d& poses,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  virtual bool calcFwdKin(Eigen::Isometry3d& pose,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          const std::string& link_name) const = 0;

  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name) const = 0;

  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;

  virtual const std::vector<std::string>& getJointNames() const = 0;

  virtual const std::vector<std::string>& getLinkNames() const = 0;

  virtual const std::vector<std::string>& getActiveLinkNames() const = 0;

  virtual const Eigen::MatrixX2d& getLimits() const = 0;

  virtual unsigned int numJoints() const = 0;

  virtual const std::string& getBaseLinkName() const = 0;

  virtual const std::string& getTipLinkName() const = 0;

  virtual const std::string& getName() const = 0;

  virtual const std::string& getSolverName() const = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<ForwardKinematicsInnerBase> clone() const = 0;
};

template <typename T>
struct ForwardKinematicsInner final : ForwardKinematicsInnerBase
{
public:
  // We just need the def ctor, delete everything else.
  ForwardKinematicsInner() = default;
  ~ForwardKinematicsInner() override = default;
  ForwardKinematicsInner(const ForwardKinematicsInner &) = delete;
  ForwardKinematicsInner(ForwardKinematicsInner &&) = delete;
  ForwardKinematicsInner &operator=(const ForwardKinematicsInner &) = delete;
  ForwardKinematicsInner &operator=(ForwardKinematicsInner &&) = delete;

  // Constructors from T (copy and move variants).
  explicit ForwardKinematicsInner(T kin) : kin_(std::move(kin)) {}
  explicit ForwardKinematicsInner(T &&kin) : kin_(std::move(kin)) {}

  std::unique_ptr<ForwardKinematicsInnerBase> clone() const override
  {
    return std::make_unique<ForwardKinematicsInner>(kin_);
  }

  bool calcFwdKin(Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override
  {
    return kin_.calcFwdKin(pose, joint_angles);
  }

  bool calcFwdKin(tesseract_common::VectorIsometry3d& poses,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override
  {
    return kin_.calcFwdKin(poses, joint_angles);
  }

  bool calcFwdKin(Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                  const std::string& link_name) const override
  {
    return kin_.calcFwdKin(pose, joint_angles, link_name);
  }

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override
  {
    return kin_.calcJacobian(jacobian, joint_angles);
  }

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                    const std::string& link_name) const override
  {
    return kin_.calcJacobian(jacobian, joint_angles, link_name);
  }

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override { return kin_.checkJoints(vec); }

  const std::vector<std::string>& getJointNames() const override { return kin_.getJointNames(); }

  const std::vector<std::string>& getLinkNames() const override { return kin_.getLinkNames(); }

  const std::vector<std::string>& getActiveLinkNames() const override { return kin_.getActiveLinkNames(); }

  const Eigen::MatrixX2d& getLimits() const override { return kin_.getLimits(); }

  unsigned int numJoints() const override { return kin_.numJoints(); }

  const std::string& getBaseLinkName() const override { return kin_.getBaseLinkName(); }

  const std::string& getTipLinkName() const override { return kin_.getBaseLinkName(); }

  const std::string& getName() const override { return kin_.getName(); }

  const std::string& getSolverName() const override { return kin_.getSolverName(); }

  T kin_;
};
}

/**
 * @brief This class represents a forward kinematics algorithm.
 *
 * Every user defined implementation must implement at least the function defined in %ForwardKinematicsInnerBase with
 * exception to the clone() which is only used internally.
*/
class ForwardKinematics
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<ForwardKinematics, uncvref_t<T>>::value, int>;

public:
  using Ptr = std::shared_ptr<ForwardKinematics>;
  using ConstPtr = std::shared_ptr<const ForwardKinematics>;

  template <typename T, generic_ctor_enabler<T> = 0>
  ForwardKinematics(T &&kin) // NOLINT
    : fwd_kin_(std::make_unique<detail::ForwardKinematicsInner<uncvref_t<T>>>(kin))
  {
  }

  // Destructor
  ~ForwardKinematics() = default;

  // Copy constructor
  ForwardKinematics(const ForwardKinematics &other) : fwd_kin_(other.fwd_kin_->clone()) {}

  // Move ctor.
  ForwardKinematics(ForwardKinematics &&other) noexcept { fwd_kin_.swap(other.fwd_kin_); }
  // Move assignment.
  ForwardKinematics &operator=(ForwardKinematics &&other) noexcept { fwd_kin_.swap(other.fwd_kin_); return (*this); }

  // Copy assignment.
  ForwardKinematics &operator=(const ForwardKinematics &other)
  {
    (*this) = ForwardKinematics(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  ForwardKinematics &operator=(T &&other)
  {
    (*this) = ForwardKinematics(std::forward<T>(other));
    return (*this);
  }

  /**
   * @brief Calculates tool pose of robot chain
   * @param pose Transform of end-of-tip relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcFwdKin(Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
    return fwd_kin_->calcFwdKin(pose, joint_angles);
  }

  /**
   * @brief Calculates pose for all links of robot chain
   * @param poses Transform of each link relative to root. Same order as getLinkNames()
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcFwdKin(tesseract_common::VectorIsometry3d& poses,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
    return fwd_kin_->calcFwdKin(poses, joint_angles);
  }

  /**
   * @brief Calculates pose for a given link
   * @param pose Transform of link relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param link_name Name of link to calculate pose which is part of the kinematics
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcFwdKin(Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                  const std::string& link_name) const
  {
    return fwd_kin_->calcFwdKin(pose, joint_angles, link_name);
  }

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian, // NOLINT
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
    return fwd_kin_->calcJacobian(jacobian, joint_angles);
  }

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @param jacobian Output jacobian for a given link
   * @param joint_angles Input vector of joint angles
   * @param link_name Name of link to calculate jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,  // NOLINT
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                    const std::string& link_name) const
  {
    return fwd_kin_->calcJacobian(jacobian, joint_angles, link_name);
  }

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const { return fwd_kin_->checkJoints(vec); }

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names, joint_list_
   */
  const std::vector<std::string>& getJointNames() const { return fwd_kin_->getJointNames(); }

  /**
   * @brief Get list of all link names (with and without geometry) for kinematic object
   * @return A vector of names, link_list_
   */
  const std::vector<std::string>& getLinkNames() const { return fwd_kin_->getLinkNames(); }

  /**
   * @brief Get list of active link names (with and without geometry) for kinematic object
   *
   * Note: This only includes links that are children of the active joints
   *
   * @return A vector of names, active_link_list_
   */
  const std::vector<std::string>& getActiveLinkNames() const { return fwd_kin_->getActiveLinkNames(); }

  /**
   * @brief Getter for joint_limits_
   * @return Matrix of joint limits
   */
  const Eigen::MatrixX2d& getLimits() const { return fwd_kin_->getLimits(); }

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  unsigned int numJoints() const { return fwd_kin_->numJoints(); }

  /** @brief getter for the robot base link name */
  const std::string& getBaseLinkName() const { return fwd_kin_->getBaseLinkName(); }

  /** @brief Get the tip link name */
  const std::string& getTipLinkName() const { return fwd_kin_->getBaseLinkName(); }

  /** @brief Name of the maniputlator */
  const std::string& getName() const { return fwd_kin_->getName(); }

  /** @brief Get the name of the solver. Recommned using the name of the class. */
  const std::string& getSolverName() const { return fwd_kin_->getSolverName(); }

private:
  std::unique_ptr<detail::ForwardKinematicsInnerBase> fwd_kin_;
};
using ForwardKinematicsPtrMap = std::unordered_map<std::string, ForwardKinematics::Ptr>;
using ForwardKinematicsConstPtrMap = std::unordered_map<std::string, ForwardKinematics::ConstPtr>;
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H
