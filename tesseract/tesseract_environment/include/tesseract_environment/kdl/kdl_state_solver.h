#ifndef TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H
#define TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/state_solver.h>

namespace tesseract_environment
{
class KDLStateSolver : public StateSolver
{
public:
  bool init(tesseract_scene_graph::SceneGraphConstPtr scene_graph) override;

  /**
   * @brief Set the current state of the solver
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names, const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names, const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  EnvStateConstPtr getCurrentState() const override { return current_state_; }

  void onEnvironmentChanged(const Commands& commands) override { createKDETree(); }

private:
  tesseract_scene_graph::SceneGraphConstPtr scene_graph_;      /**< Tesseract Scene Graph */
  EnvStatePtr current_state_;                                  /**< Current state of the environment */
  std::shared_ptr<KDL::Tree> kdl_tree_;                        /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */


  void calculateTransforms(TransformMap& transforms,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(TransformMap& transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool createKDETree();

};
typedef std::shared_ptr<KDLStateSolver> KDLStateSolverPtr;
typedef std::shared_ptr<const KDLStateSolver> KDLStateSolverConstPtr;
}
#endif // TESSERACT_ENVIRONMENT_KDL_STATE_SOLVER_H
