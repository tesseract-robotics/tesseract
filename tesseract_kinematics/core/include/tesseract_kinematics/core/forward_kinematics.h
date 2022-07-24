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
  using UPtr = std::unique_ptr<ForwardKinematics>;
  using ConstUPtr = std::unique_ptr<const ForwardKinematics>;

  ForwardKinematics() = default;
  virtual ~ForwardKinematics() = default;
  ForwardKinematics(const ForwardKinematics&) = default;
  ForwardKinematics& operator=(const ForwardKinematics&) = default;
  ForwardKinematics(ForwardKinematics&&) = default;
  ForwardKinematics& operator=(ForwardKinematics&&) = default;

  /**
   * @brief Calculates the transform for each tip link in the kinematic group
   * @details
   * This should return a transform for every link listed in getTipLinkNames()
   * Throws an exception on failures (including uninitialized)
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return A map of tip link names and transforms
   */
  virtual tesseract_common::TransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates the Jacobian matrix for a given joint state in the reference frame of the specified link
   * @details
   * This should be able to return a jacobian given any link listed in getTipLinkNames()
   * Throws an exception on failures (including uninitialized)
   * @param joint_angles Input vector of joint angles
   * @param link_name The link name to calculate jacobian
   * @return The jacobian at the provided link
   */
  virtual Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                       const std::string& link_name) const = 0;

  /** @brief Get the robot base link name */
  virtual std::string getBaseLinkName() const = 0;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Get list of tip link names for kinematic object
   * @return A vector of link names
   */
  virtual std::vector<std::string> getTipLinkNames() const = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual Eigen::Index numJoints() const = 0;

  /** @brief Get the name of the solver. Recommend using the name of the class. */
  virtual std::string getSolverName() const = 0;

  /** @brief Clone the forward kinematics object */
  virtual ForwardKinematics::UPtr clone() const = 0;
};
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H
