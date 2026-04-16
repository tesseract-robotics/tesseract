/**
 * @file kdl_state_solver.h
 * @brief Tesseract Scene Graph State Solver KDL Implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/state_solver/state_solver.h>
#include <tesseract/scene_graph/kdl_parser.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/common/kinematic_limits.h>

namespace tesseract::scene_graph
{
class KDLStateSolver : public StateSolver
{
public:
  using Ptr = std::shared_ptr<KDLStateSolver>;
  using ConstPtr = std::shared_ptr<const KDLStateSolver>;
  using UPtr = std::unique_ptr<KDLStateSolver>;
  using ConstUPtr = std::unique_ptr<const KDLStateSolver>;

  KDLStateSolver(const tesseract::scene_graph::SceneGraph& scene_graph);
  KDLStateSolver(const tesseract::scene_graph::SceneGraph& scene_graph, KDLTreeData data);
  ~KDLStateSolver() override = default;
  KDLStateSolver(const KDLStateSolver& other);
  KDLStateSolver& operator=(const KDLStateSolver& other);
  KDLStateSolver(KDLStateSolver&&) = delete;
  KDLStateSolver& operator=(KDLStateSolver&&) = delete;

  StateSolver::UPtr clone() const override;

  void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joint_values = {}) override final;
  void setState(const tesseract::common::JointIdTransformMap& floating_joint_values) override final;
  void setState(const SceneState::JointValues& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joint_values = {}) override final;
  void setState(const std::vector<tesseract::common::JointId>& joint_ids,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joint_values = {}) override final;

  SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const override final;
  SceneState getState(const tesseract::common::JointIdTransformMap& floating_joint_values) const override final;
  SceneState getState(const SceneState::JointValues& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const override final;
  SceneState getState(const std::vector<tesseract::common::JointId>& joint_ids,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const override final;

  SceneState getState() const override final;

  void getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                         const std::vector<tesseract::common::JointId>& joint_ids,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  SceneState getRandomState() const override final;

  Eigen::MatrixXd getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::LinkId& link_id,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const override final;
  Eigen::MatrixXd getJacobian(const std::vector<tesseract::common::JointId>& joint_ids,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tesseract::common::LinkId& link_id,
                               const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const override final;

  std::vector<tesseract::common::JointId> getJointIds() const override final;

  std::vector<tesseract::common::JointId> getFloatingJointIds() const override final;

  std::vector<tesseract::common::JointId> getActiveJointIds() const override final;

  tesseract::common::LinkId getBaseLinkId() const override final;

  std::vector<tesseract::common::LinkId> getLinkIds() const override final;

  std::vector<tesseract::common::LinkId> getActiveLinkIds() const override final;

  std::vector<tesseract::common::LinkId> getStaticLinkIds() const override final;

  bool isActiveLinkId(const tesseract::common::LinkId& link_id) const override final;

  bool hasLinkId(const tesseract::common::LinkId& link_id) const override final;

  tesseract::common::VectorIsometry3d getLinkTransforms() const override final;

  Eigen::Isometry3d getLinkTransform(const tesseract::common::LinkId& link_id) const override final;

  Eigen::Isometry3d getRelativeLinkTransform(const tesseract::common::LinkId& from_link_id,
                                             const tesseract::common::LinkId& to_link_id) const override final;

  tesseract::common::KinematicLimits getLimits() const override final;

private:
  SceneState current_state_;                                   /**< Current state of the environment */
  KDLTreeData data_;                                           /**< KDL tree data */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;        /**< KDL Jacobian Solver */
  std::unordered_map<tesseract::common::JointId, unsigned int, tesseract::common::JointId::Hash>
      joint_id_to_qnr_; /**< Map between joint ID and kdl q index */
  std::vector<int> joint_qnr_;                /**< The kdl segment number corresponding to joint in joint names */
  KDL::JntArray kdl_jnt_array_;               /**< The kdl joint array */
  tesseract::common::KinematicLimits limits_; /**< The kinematic limits */
  mutable std::mutex mutex_; /**< @brief KDL is not thread safe due to mutable variables in Joint Class */

  /** @brief Cached LinkId/JointId per KDL segment, avoiding per-FK constructor calls.
   *  Keyed by pointer to KDL TreeElement (pointer-stable in std::map). */
  struct SegmentIdCache
  {
    tesseract::common::LinkId link_id;
    tesseract::common::JointId joint_id;
  };
  std::unordered_map<const KDL::TreeElementType*, SegmentIdCache> segment_id_cache_;
  const KDL::TreeElementType* root_element_{ nullptr }; /**< Cached root element pointer for fast comparison */
  std::vector<tesseract::common::JointId> active_joint_ids_; /**< Cached JointIds for active joints, aligned with data_.active_joint_names */
  std::vector<tesseract::common::JointId> joint_ids_;          /**< Cached JointIds for all joints */
  std::vector<tesseract::common::JointId> floating_joint_ids_; /**< Cached JointIds for floating joints */
  std::vector<tesseract::common::LinkId> link_ids_;            /**< Cached LinkIds for all links */
  std::vector<tesseract::common::LinkId> active_link_ids_;     /**< Cached LinkIds for active links */
  std::vector<tesseract::common::LinkId> static_link_ids_;     /**< Cached LinkIds for static links */
  tesseract::common::LinkId base_link_id_;                     /**< Cached LinkId for base link */

  static thread_local KDL::JntArray kdl_joints_cache;    // NOLINT
  static thread_local KDL::Jacobian kdl_jacobian_cache;  // NOLINT

  void calculateTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                           tesseract::common::JointIdTransformMap& joint_transforms,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(tesseract::common::LinkIdTransformMap& link_transforms,
                                 tesseract::common::JointIdTransformMap& joint_transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const tesseract::common::JointId& joint_id, const double& joint_value) const;

  bool calcJacobianHelper(KDL::Jacobian& jacobian, const KDL::JntArray& kdl_joints, const tesseract::common::LinkId& link_id) const;

  bool processKDLData(const tesseract::scene_graph::SceneGraph& scene_graph);
};

}  // namespace tesseract::scene_graph
#endif  // TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
