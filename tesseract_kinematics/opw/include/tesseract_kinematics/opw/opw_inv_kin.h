/**
 * @file opw_inv_kin.h
 * @brief Tesseract OPW Inverse kinematics Wrapper
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_KINEMATICS_OPW_INV_KIN_H
#define TESSERACT_KINEMATICS_OPW_INV_KIN_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <opw_kinematics/opw_parameters.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
static const std::string OPW_INV_KIN_CHAIN_SOLVER_NAME = "OPWInvKin";

/**@brief OPW Inverse Kinematics Implementation. */
class OPWInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<OPWInvKin>;
  using ConstPtr = std::shared_ptr<const OPWInvKin>;
  using UPtr = std::unique_ptr<OPWInvKin>;
  using ConstUPtr = std::unique_ptr<const OPWInvKin>;

  ~OPWInvKin() override = default;
  OPWInvKin(const OPWInvKin& other);
  OPWInvKin& operator=(const OPWInvKin& other);
  OPWInvKin(OPWInvKin&&) = default;
  OPWInvKin& operator=(OPWInvKin&&) = default;

  /**
   * @brief Construct OPW Inverse Kinematics
   * @param params OPW kinematics parameters
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param solver_name The solver name of the kinematic chain
   */
  OPWInvKin(opw_kinematics::Parameters<double> params,
            std::string base_link_name,
            std::string tip_link_name,
            std::vector<std::string> joint_names,
            std::string solver_name = OPW_INV_KIN_CHAIN_SOLVER_NAME);

  IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  Eigen::Index numJoints() const override final;
  std::vector<std::string> getJointNames() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

protected:
  opw_kinematics::Parameters<double> params_; /**< @brief The opw kinematics parameters */
  std::string base_link_name_;                /**< @brief Link name of first link in the kinematic object */
  std::string tip_link_name_;                 /**< @brief Link name of last kink in the kinematic object */
  std::vector<std::string> joint_names_;      /**< @brief Joint names for the kinematic object */
  std::string solver_name_{ OPW_INV_KIN_CHAIN_SOLVER_NAME }; /**< @brief Name of this solver */
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_OPW_INV_KIN_H
