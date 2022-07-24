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

namespace tesseract_kinematics
{
static const std::string DEFAULT_ROP_INV_KIN_SOLVER_NAME = "ROPInvKin";

/**
 * @brief Robot on Positioner Inverse kinematic implementation.
 */
class ROPInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ROPInvKin>;
  using ConstPtr = std::shared_ptr<const ROPInvKin>;
  using UPtr = std::unique_ptr<ROPInvKin>;
  using ConstUPtr = std::unique_ptr<const ROPInvKin>;

  ~ROPInvKin() override = default;
  ROPInvKin(const ROPInvKin& other);
  ROPInvKin& operator=(const ROPInvKin& other);
  ROPInvKin(ROPInvKin&&) = default;
  ROPInvKin& operator=(ROPInvKin&&) = default;

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The Tesseract Scene State
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  ROPInvKin(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            ForwardKinematics::UPtr positioner,
            const Eigen::VectorXd& positioner_sample_resolution,
            std::string solver_name = DEFAULT_ROP_INV_KIN_SOLVER_NAME);

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The Tesseract Scene State
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_range
   * @param positioner_sample_resolution
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  ROPInvKin(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            ForwardKinematics::UPtr positioner,
            const Eigen::MatrixX2d& positioner_sample_range,
            const Eigen::VectorXd& positioner_sample_resolution,
            std::string solver_name = DEFAULT_ROP_INV_KIN_SOLVER_NAME);

  IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  std::vector<std::string> getJointNames() const override final;
  Eigen::Index numJoints() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

private:
  std::vector<std::string> joint_names_;
  InverseKinematics::UPtr manip_inv_kin_;
  ForwardKinematics::UPtr positioner_fwd_kin_;
  std::string manip_tip_link_;
  std::string positioner_tip_link_;
  double manip_reach_{ 0 };
  Eigen::Index dof_{ -1 };
  Eigen::Isometry3d positioner_to_robot_{ Eigen::Isometry3d::Identity() };
  std::vector<Eigen::VectorXd> dof_range_;
  std::string solver_name_{ DEFAULT_ROP_INV_KIN_SOLVER_NAME }; /**< @brief Name of this solver */

  void init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            ForwardKinematics::UPtr positioner,
            const Eigen::MatrixX2d& poitioner_sample_range,
            const Eigen::VectorXd& positioner_sample_resolution,
            std::string solver_name = DEFAULT_ROP_INV_KIN_SOLVER_NAME);

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const tesseract_common::TransformMap& tip_link_poses,
                               const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const tesseract_common::TransformMap& tip_link_poses,
                 Eigen::VectorXd& positioner_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const tesseract_common::TransformMap& tip_link_poses,
            Eigen::VectorXd& positioner_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H
