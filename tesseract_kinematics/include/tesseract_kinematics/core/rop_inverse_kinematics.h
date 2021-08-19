/**
 * @file rop_inverse_kinematics.h
 * @brief Robot on Positioner Inverse kinematics functions.
 *
 * @author Levi Armstrong
 * @date June 25, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::RobotOnPositionerInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief Robot on Positioner Inverse kinematic implementation.
 */
class RobotOnPositionerInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<RobotOnPositionerInvKin>;
  using ConstPtr = std::shared_ptr<const RobotOnPositionerInvKin>;
  using UPtr = std::unique_ptr<RobotOnPositionerInvKin>;
  using ConstUPtr = std::unique_ptr<const RobotOnPositionerInvKin>;

  RobotOnPositionerInvKin() = default;
  ~RobotOnPositionerInvKin() final = default;
  RobotOnPositionerInvKin(const RobotOnPositionerInvKin& other);
  RobotOnPositionerInvKin& operator=(const RobotOnPositionerInvKin& other);
  RobotOnPositionerInvKin(RobotOnPositionerInvKin&&) = default;
  RobotOnPositionerInvKin& operator=(RobotOnPositionerInvKin&&) = default;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const std::string& working_frame,
                         const std::string& link_name,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const final;

  std::vector<std::string> getJointNames() const final;
  Eigen::Index numJoints() const final;
  std::string getBaseLinkName() const final;
  std::vector<std::string> getWorkingFrames() const final;
  std::vector<std::string> getTipLinkNames() const final;
  std::string getName() const final;
  std::string getSolverName() const final;
  InverseKinematics::UPtr clone() const final;

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The Tesseract Scene State
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            ForwardKinematics::UPtr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            std::string name,
            std::string solver_name = "RobotOnPositionerInvKin");

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The Tesseract Scene State
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param poitioner_sample_range
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            ForwardKinematics::UPtr positioner,
            Eigen::MatrixX2d poitioner_sample_range,
            Eigen::VectorXd positioner_sample_resolution,
            std::string name,
            std::string solver_name = "RobotOnPositionerInvKin");

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false }; /**< @brief Identifies if the object has been initialized */
  std::vector<std::string> joint_names_;
  InverseKinematics::UPtr manip_inv_kin_;
  ForwardKinematics::UPtr positioner_fwd_kin_;
  std::string manip_tip_link_;
  std::string positioner_tip_link_;
  double manip_reach_{ 0 };
  Eigen::Index dof_;
  Eigen::Isometry3d positioner_to_robot_{ Eigen::Isometry3d::Identity() };
  std::vector<Eigen::VectorXd> dof_range_;
  std::string name_;                                     /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "RobotOnPositionerInvKin" }; /**< @brief Name of this solver */

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose,
                               const std::string& link_name,
                               const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_pose,
                 const std::string& link_name,
                 Eigen::VectorXd& positioner_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const Eigen::Isometry3d& target_pose,
            const std::string& link_name,
            Eigen::VectorXd& positioner_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H
