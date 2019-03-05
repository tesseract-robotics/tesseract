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

#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{
/** @brief Forward kinematics functions. */
class ForwardKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~ForwardKinematics() = default;
  /**
   * @brief Calculates tool pose of robot chain
   * @param pose Transform of end-of-tip relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(Eigen::Isometry3d& pose,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates pose for a given link
   * @param pose Transform of link relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param link_name Name of link to calculate pose which is part of the kinematics
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(Eigen::Isometry3d& pose,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          const std::string& link_name) const = 0;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @param jacobian Output jacobian for a given link
   * @param joint_angles Input vector of joint angles
   * @param link_name Name of link to calculate jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name) const = 0;


  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;

  /**
   * @brief Get list of joint names for robot
   * @param names Output vector of joint names, copied from joint_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual const std::vector<std::string>& getJointNames() const = 0;

  /**
   * @brief Get list of all link names (with and without geometry) for robot
   * @param names Output vector of names, copied from link_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual const std::vector<std::string>& getLinkNames() const = 0;

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

  /** @brief Name of the maniputlator */
  virtual const std::string& getName() const = 0;

};  // class BasicKin

typedef std::shared_ptr<ForwardKinematics> ForwardKinematicsPtr;
typedef std::shared_ptr<const ForwardKinematics> ForwardKinematicsConstPtr;
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_H
