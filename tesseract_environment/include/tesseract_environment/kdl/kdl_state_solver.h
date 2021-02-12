#ifndef TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H
#define TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/state_solver.h>

namespace tesseract_environment
{
class KDLStateSolver : public StateSolver
{
public:
  using Ptr = std::shared_ptr<KDLStateSolver>;
  using ConstPtr = std::shared_ptr<const KDLStateSolver>;

  KDLStateSolver() = default;
  ~KDLStateSolver() override = default;
  KDLStateSolver(const KDLStateSolver&) = delete;
  KDLStateSolver& operator=(const KDLStateSolver&) = delete;
  KDLStateSolver(KDLStateSolver&&) = delete;
  KDLStateSolver& operator=(KDLStateSolver&&) = delete;

  StateSolver::Ptr clone() const override;

  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph, int revision = 1) override;

  /**
   * @brief Set the current state of the solver
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                         const std::vector<double>& joint_values) const override;
  EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  EnvState::ConstPtr getCurrentState() const override;

  EnvState::Ptr getRandomState() const override;

  const std::vector<std::string>& getJointNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

private:
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_;    /**< Tesseract Scene Graph */
  EnvState::Ptr current_state_;                                /**< Current state of the environment */
  KDL::Tree kdl_tree_;                                         /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */
  tesseract_common::KinematicLimits limits_;                   /**< The kinematic limits */
  std::vector<std::string> joint_names_;                       /**< The active joint names */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const KDLStateSolver&);

  void calculateTransforms(EnvState& state,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(EnvState& state,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool createKDETree();

  void onEnvironmentChanged(const Commands& commands) override;
};

}  // namespace tesseract_environment
#endif  // TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H
