#ifndef TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/state_solver.h>

namespace tesseract_scene_graph
{
class KDLStateSolver : public StateSolver
{
public:
  using Ptr = std::shared_ptr<KDLStateSolver>;
  using ConstPtr = std::shared_ptr<const KDLStateSolver>;

  KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph);
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

  const SceneState& getState() const override;

  SceneState getRandomState() const override;

  const std::vector<std::string>& getJointNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

private:
  SceneState current_state_;                                   /**< Current state of the environment */
  KDL::Tree kdl_tree_;                                         /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */
  tesseract_common::KinematicLimits limits_;                   /**< The kinematic limits */
  std::vector<std::string> joint_names_;                       /**< The active joint names */

  void calculateTransforms(SceneState& state,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(SceneState& state,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool createKDETree(const tesseract_scene_graph::SceneGraph& scene_graph);
};

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_STATE_SOLVER_KDL_STATE_SOLVER_H
