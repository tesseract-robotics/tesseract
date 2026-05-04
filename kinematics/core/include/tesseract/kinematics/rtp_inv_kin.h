/**
 * @file rtp_inv_kin.h
 * @brief Robot with Tool Positioner Inverse kinematics functions.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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
#ifndef TESSERACT_KINEMATICS_RTP_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_RTP_INVERSE_KINEMATICS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/scene_graph/fwd.h>
#include <tesseract/kinematics/inverse_kinematics.h>

namespace tesseract::kinematics
{
static const std::string DEFAULT_RTP_INV_KIN_SOLVER_NAME = "RTPInvKin";
class ForwardKinematics;

/**
 * @brief Robot With Tool Positioner Inverse kinematic implementation.
 *
 * Wraps a manipulator inverse-kinematics solver and a tool-side forward-kinematics chain.
 * The tool chain's joints are sampled on a LinSpaced grid; at each sample the tool-tip target
 * is mapped back to a manipulator-tip target and the inner IK is solved.
 *
 * Base link and working frame are both the manipulator base. Tip link reported from
 * getTipLinkNames() is the tool chain tip (i.e. the frame the target pose is for).
 * Joint order in the returned solution vector: manipulator joints, then tool joints.
 *
 * @note The manipulator must have exactly one tip link; the constructor rejects multi-tip
 * manipulators since the static-offset model uses a single manipulator-tip→tool-base transform.
 */
class RTPInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<RTPInvKin>;
  using ConstPtr = std::shared_ptr<const RTPInvKin>;
  using UPtr = std::unique_ptr<RTPInvKin>;
  using ConstUPtr = std::unique_ptr<const RTPInvKin>;

  ~RTPInvKin() override = default;
  RTPInvKin(const RTPInvKin& other);
  RTPInvKin& operator=(const RTPInvKin& other);
  RTPInvKin(RTPInvKin&&) = default;
  RTPInvKin& operator=(RTPInvKin&&) = default;

  /**
   * @brief Construct RTP inverse kinematics using full tool-joint limits as the sample range.
   * @throws std::runtime_error if any input is null/invalid: scene graph root, manipulator,
   *         tool positioner, single-tip-link requirement, manipulator_reach > 0, tool joints
   *         missing or lacking limits, or tool-positioner base not rigidly attached to the
   *         manipulator tip.
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /**
   * @brief Construct RTP inverse kinematics with an explicit sample range per tool joint.
   * @throws std::runtime_error on the same conditions as the full-limits ctor, plus when
   *         @p tool_sample_range has the wrong row count, contains an inverted [min,max]
   *         pair, or @p tool_sample_resolution has a non-positive entry.
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::MatrixX2d& tool_sample_range,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /**
   * @brief Construct RTP inverse kinematics, auto-deriving manipulator_reach.
   *
   * manipulator_reach is computed as computeChainReachUpperBound() over the manipulator's
   * base->tip chain in @p scene_graph. Tool sample range defaults to full tool-joint limits.
   * @throws std::runtime_error on the same conditions as the explicit-reach ctor, and on
   *         anything propagated from computeChainReachUpperBound() (e.g. mimic joint or a
   *         joint without limits on the manipulator chain).
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /**
   * @brief Construct RTP inverse kinematics, auto-deriving manipulator_reach, explicit tool range.
   * @throws std::runtime_error on the same conditions as the explicit-range + auto-reach ctors.
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::MatrixX2d& tool_sample_range,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  void calcInvKin(IKSolutions& solutions,
                  const tesseract::common::TransformMap& tip_link_poses,
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
  std::unique_ptr<ForwardKinematics> tool_fwd_kin_;
  std::string manip_tip_link_;
  std::string tool_tip_link_;
  double manip_reach_{ 0 };
  Eigen::Isometry3d manip_tip_to_tool_base_;
  Eigen::Index dof_{ -1 };
  std::size_t grid_size_{ 1 };  /**< @brief Cached product of dof_range_[i].size(); upper-bounds inner-loop iteration count */
  std::size_t solutions_capacity_hint_{ 0 };  /**< @brief grid_size_ * 8 (OPW upper bound); reserved per IK call to avoid vector regrowth. */
  std::vector<Eigen::VectorXd> dof_range_;
  std::string solver_name_{ DEFAULT_RTP_INV_KIN_SOLVER_NAME }; /**< @brief Name of this solver */

  void init(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::MatrixX2d& tool_sample_range,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  void calcInvKinHelper(IKSolutions& solutions,
                        const tesseract::common::TransformMap& tip_link_poses,
                        const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 Eigen::Index loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_tool_tip,
                 Eigen::VectorXd& tool_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const Eigen::Isometry3d& target_tool_tip,
            Eigen::VectorXd& tool_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract::kinematics
#endif  // TESSERACT_KINEMATICS_RTP_INVERSE_KINEMATICS_H
