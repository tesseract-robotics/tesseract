/**
 * @file kdl_state_solver.h
 * @brief Tesseract Scene Graph State Solver KDL Implementation.
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
#ifndef TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/state_solver.h>
#include <tesseract_scene_graph/kdl_parser.h>

namespace tesseract_scene_graph
{
class KDLStateSolver : public StateSolver
{
public:
  using Ptr = std::shared_ptr<KDLStateSolver>;
  using ConstPtr = std::shared_ptr<const KDLStateSolver>;
  using UPtr = std::unique_ptr<KDLStateSolver>;
  using ConstUPtr = std::unique_ptr<const KDLStateSolver>;

  KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph);
  KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph, KDLTreeData data);
  ~KDLStateSolver() override = default;
  KDLStateSolver(const KDLStateSolver& other);
  KDLStateSolver& operator=(const KDLStateSolver& other);
  KDLStateSolver(KDLStateSolver&&) = delete;
  KDLStateSolver& operator=(KDLStateSolver&&) = delete;

  StateSolver::UPtr clone() const override;

  void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) override final;
  void setState(const std::unordered_map<std::string, double>& joint_values) override final;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override final;

  SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;
  SceneState getState(const std::unordered_map<std::string, double>& joint_values) const override final;
  SceneState getState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  SceneState getState() const override final;

  SceneState getRandomState() const override final;

  Eigen::MatrixXd getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override final;

  Eigen::MatrixXd getJacobian(const std::unordered_map<std::string, double>& joint_values,
                              const std::string& link_name) const override final;
  Eigen::MatrixXd getJacobian(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override final;

  std::vector<std::string> getJointNames() const override final;

  std::vector<std::string> getActiveJointNames() const override final;

  std::string getBaseLinkName() const override final;

  std::vector<std::string> getLinkNames() const override final;

  std::vector<std::string> getActiveLinkNames() const override final;

  std::vector<std::string> getStaticLinkNames() const override final;

  bool isActiveLinkName(const std::string& link_name) const override final;

  bool hasLinkName(const std::string& link_name) const override final;

  tesseract_common::VectorIsometry3d getLinkTransforms() const override final;

  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const override final;

  Eigen::Isometry3d getRelativeLinkTransform(const std::string& from_link_name,
                                             const std::string& to_link_name) const override final;

  tesseract_common::KinematicLimits getLimits() const override final;

private:
  SceneState current_state_;                                   /**< Current state of the environment */
  KDLTreeData data_;                                           /**< KDL tree data */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;        /**< KDL Jacobian Solver */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  std::vector<int> joint_qnr_;               /**< The kdl segment number corresponding to joint in joint names */
  KDL::JntArray kdl_jnt_array_;              /**< The kdl joint array */
  tesseract_common::KinematicLimits limits_; /**< The kinematic limits */
  mutable std::mutex mutex_; /**< @brief KDL is not thread safe due to mutable variables in Joint Class */

  void calculateTransforms(SceneState& state,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(SceneState& state,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool calcJacobianHelper(KDL::Jacobian& jacobian, const KDL::JntArray& kdl_joints, const std::string& link_name) const;

  /** @brief Get an updated kdl joint array */
  KDL::JntArray getKDLJntArray(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
  KDL::JntArray getKDLJntArray(const std::unordered_map<std::string, double>& joint_values) const;

  bool processKDLData(const tesseract_scene_graph::SceneGraph& scene_graph);
};

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
