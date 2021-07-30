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

#ifdef SWIG
%shared_ptr(tesseract_kinematics::OPWInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**@brief OPW Inverse Kinematics Implmentation. */
class OPWInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<OPWInvKin>;
  using ConstPtr = std::shared_ptr<const OPWInvKin>;

  OPWInvKin() = default;
  ~OPWInvKin() override = default;
  OPWInvKin(const OPWInvKin&) = delete;
  OPWInvKin& operator=(const OPWInvKin&) = delete;
  OPWInvKin(OPWInvKin&&) = delete;
  OPWInvKin& operator=(OPWInvKin&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool update() override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                         const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;
  unsigned int numJoints() const override;

  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;
  const std::vector<std::string>& getActiveLinkNames() const override;
  const tesseract_common::KinematicLimits& getLimits() const override;
  void setLimits(tesseract_common::KinematicLimits limits) override;
  std::vector<Eigen::Index> getRedundancyCapableJointIndices() const override;
  const std::string& getBaseLinkName() const override;
  const std::string& getTipLinkName() const override;
  const std::string& getName() const override;
  const std::string& getSolverName() const override;

  /**
   * @brief init Initialize OPW Inverse Kinematics
   * @param name The name of the kinematic chain
   * @param params OPW kinematics parameters
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param link_names The link names for the kinematic chain
   * @param active_link_names The active links names for the kinematic chain
   * @param joint_limits The joint limits for the kinematic chain
   * @return True if successful
   */
  bool init(std::string name,
            opw_kinematics::Parameters<double> params,
            std::string base_link_name,
            std::string tip_link_name,
            std::vector<std::string> joint_names,
            std::vector<std::string> link_names,
            std::vector<std::string> active_link_names,
            tesseract_common::KinematicLimits limits);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

protected:
  bool initialized_{ false };                  /**< @brief Identifies if the object has been initialized */
  opw_kinematics::Parameters<double> params_;  /**< @brief The opw kinematics parameters */
  std::string base_link_name_;                 /**< @brief Kinematic base link name */
  std::string tip_link_name_;                  /**< @brief Kinematic tip link name */
  tesseract_common::KinematicLimits limits_;   /**< @brief Joint Limits, velocity limits, and acceleration limits */
  std::vector<std::string> joint_names_;       /**< @brief joint names */
  std::vector<std::string> link_names_;        /**< @brief link names */
  std::vector<std::string> active_link_names_; /**< @brief active link names */
  std::string name_;                           /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "OPWInvKin" };     /**< @brief Name of this solver */
  std::vector<Eigen::Index> redundancy_indices_{ 0, 1, 2, 3, 4, 5 }; /**< Joint indicies that have redundancy (ex.
                                                                        revolute) */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const OPWInvKin& kin);
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_OPW_INV_KIN_H
