/**
 * @file ikfast_inv_kin.h
 * @brief Tesseract IKFast Inverse kinematics Wrapper
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
#ifndef TESSERACT_KINEMATICS_IKFAST_INV_KIN_H
#define TESSERACT_KINEMATICS_IKFAST_INV_KIN_H

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
static const std::string IKFAST_INV_KIN_CHAIN_SOLVER_NAME = "IKFastInvKin";

/**
 * @brief IKFast Inverse Kinematics Implmentation.
 *
 * This along with the ikfast_inv_kin.hpp is to be used with a generated ikfast to create a tesseract implementation.
 * Once you have created your ikfast solver of your robot all you need is to create header similar to what is shown
 * below:
 *
 * Header File: fanuc_p50ib_15_inv_kinematics.h
 *
 * #include <Eigen/Geometry>
 * #include <vector>
 * #include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
 *
 * namespace fanuc_p50ib_15_ikfast_wrapper
 * {
 * class FanucP50iBInvKinematics : public tesseract_kinematics::IKFastInvKin
 * {
 * public:
 *   FanucP50iBInvKinematics(const std::string base_link_name,
 *                           const std::string tip_link_name,
 *                           const std::vector<std::string> joint_names,
 *                           const std::string name)
 * };
 *
 * Cpp File:
 *
 * #include <tesseract_kinematics/ikfast/impl/ikfast_inv_kin.hpp>
 * #include <fanuc_p50ib_15_ikfast_wrapper/impl/fanuc_p50ib_15_ikfast.hpp>
 * #include <fanuc_p50ib_15_ikfast_wrapper/tesseract_fanuc_p50ib_kinematics.h>
 *
 * namespace fanuc_p50ib_15_ikfast_wrapper
 * {
 *   FanucP50iBInvKinematics::FanucP50iBInvKinematics(const std::string base_link_name,
 *                                                    const std::string tip_link_name,
 *                                                    const std::vector<std::string> joint_names
 *                                                    const std::string name)
 *   : FanucP50iBInvKinematics(base_link_name, tip_link_name, joint_names, name, joint_limits)
 *   {}
 * }
 *
 */
class IKFastInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<IKFastInvKin>;
  using ConstPtr = std::shared_ptr<const IKFastInvKin>;
  using UPtr = std::unique_ptr<IKFastInvKin>;
  using ConstUPtr = std::unique_ptr<const IKFastInvKin>;

  ~IKFastInvKin() override = default;
  IKFastInvKin(const IKFastInvKin& other);
  IKFastInvKin& operator=(const IKFastInvKin& other);
  IKFastInvKin(IKFastInvKin&&) = default;
  IKFastInvKin& operator=(IKFastInvKin&&) = default;

  /**
   * @brief Construct IKFast Inverse Kinematics
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param solver_name The solver name of the kinematic chain
   */
  IKFastInvKin(std::string base_link_name,
               std::string tip_link_name,
               std::vector<std::string> joint_names,
               std::vector<Eigen::Index> redundancy_capable_joints,
               std::string solver_name = IKFAST_INV_KIN_CHAIN_SOLVER_NAME,
               std::vector<std::vector<double>> free_joint_states = {});

  IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  Eigen::Index numJoints() const override;
  std::vector<std::string> getJointNames() const override;
  std::string getBaseLinkName() const override;
  std::string getWorkingFrame() const override;
  std::vector<std::string> getTipLinkNames() const override;
  std::string getSolverName() const override;
  InverseKinematics::UPtr clone() const override;

  /**
   * @brief Generates all possible combinations of joint states and stores it to the free_joint_states_ class member
   * Example: Given 2 free joints, wanting to sample the first joint at 0, 1, and 2 and the second joint at 3 and 4
   * the input would be [[0, 1, 2][3,4]] and it would generate [[0,3][0,4][1,3][1,4][2,3][2,4]]
   * @param free_joint_samples A vector of vectors in which the lower level vectors each represent all of a single
   * joint's possible positions to be sampled
   * @return
   */
  static std::vector<std::vector<double>>
  generateAllFreeJointStateCombinations(const std::vector<std::vector<double>>& free_joint_samples);

protected:
  std::string base_link_name_;                          /**< @brief Link name of first link in the kinematic object */
  std::string tip_link_name_;                           /**< @brief Link name of last kink in the kinematic object */
  std::vector<std::string> joint_names_;                /**< @brief Joint names for the kinematic object */
  std::vector<Eigen::Index> redundancy_capable_joints_; /** @brief Redundancy capable joints */
  std::string solver_name_{ IKFAST_INV_KIN_CHAIN_SOLVER_NAME }; /**< @brief Name of this solver */
  /**< @brief combinations of free joints to sample when computing IK
   * Example: Given 3 free joints, a valid input would be [[0,0,0][0,0,1][-1,0,1][0,2,0]] */
  std::vector<std::vector<double>> free_joint_states_;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_IKFAST_INV_KIN_H
