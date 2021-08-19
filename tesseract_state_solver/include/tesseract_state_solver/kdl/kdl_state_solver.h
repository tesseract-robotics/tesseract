#ifndef TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
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

  KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph);
  KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph, KDLTreeData data);
  ~KDLStateSolver() override = default;
  KDLStateSolver(const KDLStateSolver& other);
  KDLStateSolver& operator=(const KDLStateSolver& other);
  KDLStateSolver(KDLStateSolver&&) = default;
  KDLStateSolver& operator=(KDLStateSolver&&) = default;

  StateSolver::UPtr clone() const override;

  void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;
  SceneState getState(const std::unordered_map<std::string, double>& joints) const override;
  SceneState getState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  SceneState getState() const override;

  SceneState getRandomState() const override;

  Eigen::MatrixXd getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override;

  Eigen::MatrixXd getJacobian(const std::unordered_map<std::string, double>& joints,
                              const std::string& link_name) const override;
  Eigen::MatrixXd getJacobian(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override;

  std::vector<std::string> getJointNames() const override;

  std::string getBaseLinkName() const override;

  std::vector<std::string> getLinkNames() const override;

  std::vector<std::string> getActiveLinkNames() const override;

  tesseract_common::KinematicLimits getLimits() const override;

private:
  SceneState current_state_;                                   /**< Current state of the environment */
  KDLTreeData data_;                                           /**< KDL tree data */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;        /**< KDL Jacobian Solver */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  std::vector<int> joint_qnr_;               /**< The kdl segment number corrisponding to joint in joint names */
  KDL::JntArray kdl_jnt_array_;              /**< The kdl joint array */
  tesseract_common::KinematicLimits limits_; /**< The kinematic limits */

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
  KDL::JntArray getKDLJntArray(const std::unordered_map<std::string, double>& joints) const;

  bool processKDLData(const tesseract_scene_graph::SceneGraph& scene_graph);
};

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
