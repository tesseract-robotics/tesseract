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
#include <tesseract_kinematics/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::InverseKinematics)
%unique_ptr(tesseract_kinematics::InverseKinematics)
#endif  // SWIG

namespace tesseract_kinematics
{
/** @brief Inverse kinematics functions. */
class InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<InverseKinematics>;
  using ConstPtr = std::shared_ptr<const InverseKinematics>;
  using UPtr = std::unique_ptr<InverseKinematics>;
  using ConstUPtr = std::unique_ptr<const InverseKinematics>;

  InverseKinematics() = default;
  virtual ~InverseKinematics() = default;
  InverseKinematics(const InverseKinematics&) = default;
  InverseKinematics& operator=(const InverseKinematics&) = default;
  InverseKinematics(InverseKinematics&&) = default;
  InverseKinematics& operator=(InverseKinematics&&) = default;

  /**
   * @brief Calculates joint solutions given a pose for a specific link.
   * @details This is to support a pose relative to a active link. For example a robot
   * with an external positioner where the pose is relative to the tip link of the positioner.
   * @note If redundant solutions are needed see utility funciton getRedundantSolutions.
   * @param pose Transform of end-of-tip relative to working_frame
   * @param working_frame The link name the pose is relative to. It must be listed in getTipLinkNames().
   * @param tip_link_name The tip link to use for solving inverse kinematics. It must be listed in getTipLinkNames().
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  virtual IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                                 const std::string& working_frame,
                                 const std::string& tip_link_name,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed) const = 0;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names, joint_list_
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual Eigen::Index numJoints() const = 0;

  /** @brief getter for the robot base link name */
  virtual std::string getBaseLinkName() const = 0;

  /** @brief getter for robot working frames */
  virtual std::vector<std::string> getWorkingFrames() const = 0;

  /** @brief Get the tip link name */
  virtual std::vector<std::string> getTipLinkNames() const = 0;

  /** @brief Name of the maniputlator */
  virtual std::string getName() const = 0;

  /** @brief Get the name of the solver. Recommned using the name of the class. */
  virtual std::string getSolverName() const = 0;

  /** @brief Clone the forward kinematics object */
  virtual std::unique_ptr<InverseKinematics> clone() const = 0;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_INVERSE_KINEMATICS_H
