#ifndef TESSERACT_KINEMATICS_STATIC_KINEMATIC_GROUP_H
#define TESSERACT_KINEMATICS_STATIC_KINEMATIC_GROUP_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_scene_graph/adjacency_map.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_kinematics
{
/**
 * @brief A kinematic group with assumes all objects are state except for the links manipulated by the kinematics object
 * @details This is primarilly used by motion planning, but if you are wanting to account for a dynamic world use the
 * DynamicKinematicGroup
 */
class StaticKinematicGroup : public KinematicGroup
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<StaticKinematicGroup>;
  using ConstPtr = std::shared_ptr<const StaticKinematicGroup>;
  using UPtr = std::unique_ptr<StaticKinematicGroup>;
  using ConstUPtr = std::unique_ptr<const StaticKinematicGroup>;

  virtual ~StaticKinematicGroup() final = default;
  StaticKinematicGroup(const StaticKinematicGroup& other);
  StaticKinematicGroup& operator=(const StaticKinematicGroup& other);
  StaticKinematicGroup(StaticKinematicGroup&&) = default;
  StaticKinematicGroup& operator=(StaticKinematicGroup&&) = default;

  StaticKinematicGroup(InverseKinematics::UPtr inv_kin,
                       const tesseract_scene_graph::SceneGraph& scene_graph,
                       const tesseract_scene_graph::SceneState& scene_state);

  tesseract_common::TransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const final;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const std::string& working_frame,
                         const std::string& tip_link_name,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const final;

  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name = "") const final;

  const std::vector<std::string>& getJointNames() const final;

  const std::vector<std::string>& getLinkNames() const final;

  const std::vector<std::string>& getActiveLinkNames() const final;

  const tesseract_common::KinematicLimits& getLimits() const final;

  void setLimits(tesseract_common::KinematicLimits limits) final;

  const std::vector<Eigen::Index>& getRedundancyCapableJointIndices() const final;

  Eigen::Index numJoints() const final;

  const std::string& getBaseLinkName() const final;

  const std::vector<std::string>& getWorkingFrames() const final;

  const std::vector<std::string>& getTipLinkNames() const final;

  const std::string& getName() const final;

  std::unique_ptr<KinematicGroup> clone() const final;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const final;

private:
  std::string name_;
  tesseract_scene_graph::SceneState state_;
  tesseract_scene_graph::KDLStateSolver::UPtr state_solver_;
  InverseKinematics::UPtr inv_kin_;
  Eigen::Isometry3d inv_to_fwd_base_{ Eigen::Isometry3d::Identity() };
  std::vector<std::string> joint_names_;
  std::vector<std::string> working_frames_;
  std::vector<std::string> tip_link_names_;
  std::vector<std::string> static_link_names_;
  tesseract_common::KinematicLimits limits_;
  std::vector<Eigen::Index> redundancy_indices_;
  std::unordered_map<std::string, std::string> inv_working_frames_map_;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_STATIC_KINEMATIC_GROUP_H
