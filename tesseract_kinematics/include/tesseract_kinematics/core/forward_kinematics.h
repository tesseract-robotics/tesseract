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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_scene_graph/graph.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::ForwardKinematics)
%template(ForwardKinematicsPtrMap) std::unordered_map<std::string, tesseract_kinematics::ForwardKinematics::Ptr>;
%template(ForwardKinematicsConstPtrMap) std::unordered_map<std::string, tesseract_kinematics::ForwardKinematics::ConstPtr>;
#endif  // SWIG

namespace tesseract_kinematics
{
/** @brief Forward kinematics functions. */
class ForwardKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ForwardKinematics>;
  using ConstPtr = std::shared_ptr<const ForwardKinematics>;

  ForwardKinematics() = default;
  virtual ~ForwardKinematics() = default;
  ForwardKinematics(const ForwardKinematics&) = delete;
  ForwardKinematics& operator=(const ForwardKinematics&) = delete;
  ForwardKinematics(ForwardKinematics&&) = delete;
  ForwardKinematics& operator=(ForwardKinematics&&) = delete;

  /**
   * @brief Updates kinematics if kinematic parameters have changed
   * @return True if successful
   */
  virtual bool update() = 0;

  /**
   * @brief Calculates tool pose of robot chain
   * @details Throws an exception on failures (including uninitialized)
   * @param pose Transform of end-of-tip relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   */
  virtual Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates pose for all links of robot chain
   * @details Throws an exception on failures (including uninitialized)
   * @param poses Transform of each link relative to root. Same order as getLinkNames()
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   */
  virtual tesseract_common::VectorIsometry3d
  calcFwdKinAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates pose for a given link
   * @details Throws an exception on failures (including uninitialized)
   * @param pose Transform of link relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param link_name Name of link to calculate pose which is part of the kinematics
   */
  virtual Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                       const std::string& link_name) const = 0;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @details Throws an exception on failures (including uninitialized)
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   */
  virtual Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @details Throws an exception on failures (including uninitialized)
   * @param jacobian Output jacobian for a given link
   * @param joint_angles Input vector of joint angles
   * @param link_name Name of link to calculate jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                       const std::string& link_name) const = 0;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names, joint_list_
   */
  virtual const std::vector<std::string>& getJointNames() const = 0;

  /**
   * @brief Get list of all link names (with and without geometry) for kinematic object
   * @return A vector of names, link_list_
   */
  virtual const std::vector<std::string>& getLinkNames() const = 0;

  /**
   * @brief Get list of active link names (with and without geometry) for kinematic object
   *
   * Note: This only includes links that are children of the active joints
   *
   * @return A vector of names, active_link_list_
   */
  virtual const std::vector<std::string>& getActiveLinkNames() const = 0;

  /**
   * @brief Getter for kinematic limits (joint, velocity, acceleration, etc.)
   * @return Kinematic Limits
   */
  virtual const tesseract_common::KinematicLimits& getLimits() const = 0;

  /**
   * @brief Setter for kinematic limits (joint, velocity, acceleration, etc.)
   * @param Kinematic Limits
   */
  virtual void setLimits(tesseract_common::KinematicLimits limits) = 0;

  /**
   * @brief Get vector indicating which joints are capable of producing redundant solutions
   * @return A vector of joint indicies
   */
  virtual std::vector<Eigen::Index> getRedundancyCapableJointIndices() const = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual unsigned int numJoints() const = 0;

  /** @brief getter for the robot base link name */
  virtual const std::string& getBaseLinkName() const = 0;

  /** @brief Get the tip link name */
  virtual const std::string& getTipLinkName() const = 0;

  /** @brief Name of the maniputlator */
  virtual const std::string& getName() const = 0;

  /** @brief Get the name of the solver. Recommned using the name of the class. */
  virtual const std::string& getSolverName() const = 0;

  /** @brief Clone the forward kinematics object */
  virtual std::shared_ptr<ForwardKinematics> clone() const = 0;
};

using ForwardKinematicsPtrMap = std::unordered_map<std::string, ForwardKinematics::Ptr>;
using ForwardKinematicsConstPtrMap = std::unordered_map<std::string, ForwardKinematics::ConstPtr>;
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H
