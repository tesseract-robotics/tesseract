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
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/eigen_types.h>
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
 * The tool chain's contribution at each grid sample depends only on the grid, so it is evaluated
 * once during construction and stored (@ref sample_to_manip_tip_). calcInvKin() therefore never
 * calls the tool forward kinematics, and the class does not retain it after construction.
 *
 * Base link and working frame are both the manipulator base. Tip link reported from
 * getTipLinkNames() is the tool chain tip (i.e. the frame the target pose is for).
 * Joint order in the returned solution vector: manipulator joints, then tool joints.
 *
 * @note The manipulator and the tool positioner must each have exactly one tip link; the
 * constructor rejects anything else. The static-offset model needs a single manipulator-tip→
 * tool-base transform, and the tool tip is the frame target poses are interpreted in.
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
   *
   * All four constructors share one contract, enforced in a single place; they differ only in
   * whether @p manipulator_reach is supplied or derived, and whether the tool sample range is
   * supplied or taken from the tool joints' full limits.
   *
   * @throws std::runtime_error if any input is null or invalid: null manipulator or tool
   *         positioner, empty solver name, invalid scene graph root, a manipulator or tool
   *         positioner with other than exactly one tip link, a manipulator reach that is not
   *         greater than zero, a
   *         tool sample range or resolution whose size disagrees with the tool joint count, a
   *         malformed sample range/resolution pair (see buildSampleGrid()), a combined sample
   *         grid above the supported size, a tool-positioner base that is not rigidly attached to
   *         the manipulator tip, or a @p scene_state missing a transform for the manipulator tip
   *         or tool-positioner base.
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            double manipulator_reach,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /**
   * @brief As above, but with an explicit sample range per tool joint instead of the full
   *        tool-joint limits. @p tool_sample_range must have one row per tool joint.
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
   * @brief As the full-limits ctor, but auto-deriving manipulator_reach as
   *        computeChainReachUpperBound() over the manipulator's base->tip chain.
   * @throws std::runtime_error additionally on anything propagated from
   *         computeChainReachUpperBound() (e.g. a mimic joint, or a joint without limits, on
   *         the manipulator chain).
   */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /** @brief As above, auto-deriving manipulator_reach and taking an explicit tool sample range. */
  RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const Eigen::MatrixX2d& tool_sample_range,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name = DEFAULT_RTP_INV_KIN_SOLVER_NAME);

  /**
   * @brief See InverseKinematics::calcInvKin().
   * @throws std::out_of_range if @p tip_link_poses has no entry for getTipLinkNames()[0]. That is
   *         a precondition of the interface, not a runtime failure, so it is asserted in debug
   *         builds and left to the map lookup otherwise.
   */
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
  std::string manip_tip_link_;
  std::string tool_tip_link_;
  double manip_reach_{ 0 };
  Eigen::Index dof_{ -1 };

  /** @brief Tool joint values at each grid sample; one column per sample, one row per tool joint. */
  Eigen::MatrixXd tool_samples_;

  /**
   * @brief Per-sample transform taking a tool-tip target to the corresponding manipulator-tip
   *        target, i.e. (T_manip_tip→tool_base * T_tool_base→tool_tip(q_k))⁻¹ for sample k.
   * @details Column k of tool_samples_ and entry k here describe the same grid sample. Both are
   *          built once in init(), so calcInvKin() reduces to one transform product per sample.
   */
  tesseract::common::VectorIsometry3d sample_to_manip_tip_;

  std::string solver_name_{ DEFAULT_RTP_INV_KIN_SOLVER_NAME }; /**< @brief Name of this solver */

  /**
   * @brief Validates every input and populates all members; the four public constructors are
   *        one-line delegations to it.
   * @param manipulator_reach  Empty to derive it from the manipulator's base->tip chain.
   * @param tool_sample_range  Empty to use the tool joints' full limits.
   */
  void init(const tesseract::scene_graph::SceneGraph& scene_graph,
            const tesseract::scene_graph::SceneState& scene_state,
            InverseKinematics::UPtr manipulator,
            std::optional<double> manipulator_reach,
            std::unique_ptr<ForwardKinematics> tool_positioner,
            const std::optional<Eigen::MatrixX2d>& tool_sample_range,
            const Eigen::VectorXd& tool_sample_resolution,
            std::string solver_name);
};
}  // namespace tesseract::kinematics
#endif  // TESSERACT_KINEMATICS_RTP_INVERSE_KINEMATICS_H
