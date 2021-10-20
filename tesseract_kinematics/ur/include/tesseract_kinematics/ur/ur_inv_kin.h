/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef TESSERACT_KINEMATICS_UR_INV_KIN_H
#define TESSERACT_KINEMATICS_UR_INV_KIN_H

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
static const std::string UR_INV_KIN_CHAIN_SOLVER_NAME = "URInvKin";

/**@brief Universal Robot Inverse Kinematics Implementation. */
class URInvKin : public tesseract_kinematics::InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<URInvKin>;
  using ConstPtr = std::shared_ptr<const URInvKin>;
  using UPtr = std::unique_ptr<URInvKin>;
  using ConstUPtr = std::unique_ptr<const URInvKin>;

  ~URInvKin() override = default;
  URInvKin(const URInvKin& other);
  URInvKin& operator=(const URInvKin& other);
  URInvKin(URInvKin&&) = default;
  URInvKin& operator=(URInvKin&&) = default;

  /**
   * @brief init Initialize UR Inverse Kinematics
   * @param params UR kinematics parameters
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param solver_name The solver name of the kinematic chain
   */
  URInvKin(URParameters params,
           std::string base_link_name,
           std::string tip_link_name,
           std::vector<std::string> joint_names,
           std::string solver_name = UR_INV_KIN_CHAIN_SOLVER_NAME);

  tesseract_kinematics::IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                               const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  Eigen::Index numJoints() const override final;
  std::vector<std::string> getJointNames() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

protected:
  URParameters params_;                  /**< @brief The UR Inverse kinematics parameters */
  std::string base_link_name_;           /**< @brief Link name of first link in the kinematic object */
  std::string tip_link_name_;            /**< @brief Link name of last kink in the kinematic object */
  std::vector<std::string> joint_names_; /**< @brief Joint names for the kinematic object */
  std::string solver_name_{ UR_INV_KIN_CHAIN_SOLVER_NAME }; /**< @brief Name of this solver */
};
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_UR_INV_KIN_H
