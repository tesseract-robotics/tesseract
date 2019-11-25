/**
 * @file inverse_kinematics.h
 * @brief Inverse kinematics functions.
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
#ifndef TESSERACT_KINEMATICS_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_INVERSE_KINEMATICS_H

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

namespace tesseract_kinematics
{
/** @brief Inverse kinematics functions. */
class InverseKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<InverseKinematics>;
  using ConstPtr = std::shared_ptr<const InverseKinematics>;

  InverseKinematics() = default;
  virtual ~InverseKinematics() = default;
  InverseKinematics(const InverseKinematics&) = delete;
  InverseKinematics& operator=(const InverseKinematics&) = delete;
  InverseKinematics(InverseKinematics&&) = delete;
  InverseKinematics& operator=(InverseKinematics&&) = delete;

  /**
   * @brief Calculates joint solutions given a pose.
   * @param solutions A vector of solutions, so check the size of the vector to determine the number of solutions
   * @param pose Transform of end-of-tip relative to root (base link)
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized)
   */
  virtual bool calcInvKin(Eigen::VectorXd& solutions,
                          const Eigen::Isometry3d& pose,
                          const Eigen::Ref<const Eigen::VectorXd>& seed) const = 0;

  /**
   * @brief Calculates joint solutions given a pose for a specific link.
   * @param solutions A vector of solutions, so check the size of the vector to determine the number of solutions
   * @param pose Transform of end-of-tip relative to root (base link)
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized)
   */
  virtual bool calcInvKin(Eigen::VectorXd& solutions,
                          const Eigen::Isometry3d& pose,
                          const Eigen::Ref<const Eigen::VectorXd>& seed,
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
   * @brief Getter for joint_limits_
   * @return Matrix of joint limits
   */
  virtual const Eigen::MatrixX2d& getLimits() const = 0;

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
  virtual std::shared_ptr<InverseKinematics> clone() const = 0;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_INVERSE_KINEMATICS_H
