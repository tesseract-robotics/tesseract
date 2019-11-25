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

namespace tesseract_kinematics
{
/**
 * @brief IKFast Inverse Kinematics Implmentation.
 *
 * This along with the ikfast_inv_kin.hpp is to be used with a generated ikfast to create a tesseract implementation.
 * Once you have created your ikfast solver of your robot all you need is to create header similiar to what is shown
 * below:
 *
 * Header File: fanuc_p50ib_15_inv_kinematics.h
 *
 * #include <Eigen/Geometry>
 * #include <vector>
 * #include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
 * #include <tesseract_kinematics/core/forward_kinematics.h>

 * namespace fanuc_p50ib_15_ikfast_wrapper
 * {
 * class FanucP50iBInvKinematics : public tesseract_kinematics::IKFastInvKin
 * {
 * public:
 *   FanucP50iBInvKinematics(const std::string name,
 *                           const std::string base_link_name,
 *                           const std::string tip_link_name,
 *                           const std::vector<std::string> joint_names,
 *                           const std::vector<std::string> link_names,
 *                           const std::vector<std::string> active_link_names,
 *                           const Eigen::MatrixX2d joint_limits)
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
 *   FanucP50iBInvKinematics::FanucP50iBInvKinematics(const std::string name,
 *                                                    const std::string base_link_name,
 *                                                    const std::string tip_link_name,
 *                                                    const std::vector<std::string> joint_names,
 *                                                    const std::vector<std::string> link_names,
 *                                                    const std::vector<std::string> active_link_names,
 *                                                    const Eigen::MatrixX2d joint_limits)
 *   : FanucP50iBInvKinematics(name, base_link_name, tip_link_name, joint_names, link_names, active_link_names,
 joint_limits)
 *   {}
 * }
 *
*/
class IKFastInvKin : public InverseKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<IKFastInvKin>;
  using ConstPtr = std::shared_ptr<const IKFastInvKin>;

  IKFastInvKin() : initialized_(false), solver_name_("IKFastInvKin") {}
  IKFastInvKin(const IKFastInvKin&) = delete;
  IKFastInvKin& operator=(const IKFastInvKin&) = delete;
  IKFastInvKin(IKFastInvKin&&) = delete;
  IKFastInvKin& operator=(IKFastInvKin&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed,
                  const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;
  unsigned int numJoints() const override;

  const std::vector<std::string>& getJointNames() const override { return joint_names_; }
  const std::vector<std::string>& getLinkNames() const override { return link_names_; }
  const std::vector<std::string>& getActiveLinkNames() const override { return active_link_names_; }
  const Eigen::MatrixX2d& getLimits() const override { return joint_limits_; }
  const std::string& getBaseLinkName() const override { return base_link_name_; }
  const std::string& getTipLinkName() const override { return tip_link_name_; }
  const std::string& getName() const override { return name_; }
  const std::string& getSolverName() const override { return solver_name_; }

  /**
   * @brief Initialize IKFast Inverse Kinematics
   * @param name The name of the kinematic chain
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param link_names The link names for the kinematic chain
   * @param active_link_names The active links names for the kinematic chain
   * @param joint_limits The joint limits for the kinematic chain
   * @return True if successful
   */
  bool init(const std::string name,
            const std::string base_link_name,
            const std::string tip_link_name,
            const std::vector<std::string> joint_names,
            const std::vector<std::string> link_names,
            const std::vector<std::string> active_link_names,
            const Eigen::MatrixX2d joint_limits);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const
  {
    if (!initialized_)
    {
      CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
    }

    return initialized_;
  }

protected:
  bool initialized_ = false;                   /**< @brief Identifies if the object has been initialized */
  std::string base_link_name_;                 /**< @brief Kinematic base link name */
  std::string tip_link_name_;                  /**< @brief Kinematic tip link name */
  Eigen::MatrixX2d joint_limits_;              /**< @brief Joint Limits */
  std::vector<std::string> joint_names_;       /**< @brief joint names */
  std::vector<std::string> link_names_;        /**< @brief link names */
  std::vector<std::string> active_link_names_; /**< @brief active link names */
  std::string name_;                           /**< @brief Name of the kinematic chain */
  std::string solver_name_;                    /**< @brief Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const IKFastInvKin& kin);
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_IKFAST_INV_KIN_H
