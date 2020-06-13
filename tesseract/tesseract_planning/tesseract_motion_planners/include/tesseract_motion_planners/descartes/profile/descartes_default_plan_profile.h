#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/types.h>
#include <Eigen/Geometry>

namespace tesseract_planning
{

template <typename FloatType>
class DescartesDefaultPlanProfile : public DescartesPlanProfile<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesDefaultPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesDefaultPlanProfile<FloatType>>;

  PoseSamplerFn target_pose_sampler;
  Eigen::VectorXd positioner_sample_resolution; // TODO Does this belong here or in the problem?
  bool enable_collision {false};
  bool allow_collision {false};
  double collision_safety_margin {0};
  DescartesIsValidFn<FloatType> is_valid;
  bool debug {false};

  void apply(DescartesProblem<FloatType>& prob,
             const Eigen::Isometry3d& cartesian_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;

  void apply(DescartesProblem<FloatType>& prob,
             const Eigen::VectorXd& joint_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;

protected:
  void applyRobotOnly(DescartesProblem<FloatType>& prob,
                      const Eigen::Isometry3d& cartesian_waypoint,
                      const PlanInstruction& parent_instruction,
                      const std::vector<std::string> &active_links,
                      int index);

  void applyRobotOnly(DescartesProblem<FloatType>& prob,
                      const Eigen::VectorXd& joint_waypoint,
                      const PlanInstruction& parent_instruction,
                      const std::vector<std::string> &active_links,
                      int index);

  void applyRobotOnPositioner(DescartesProblem<FloatType>& prob,
                              const Eigen::Isometry3d& cartesian_waypoint,
                              const PlanInstruction& parent_instruction,
                              const std::vector<std::string> &active_links,
                              int index);

  void applyRobotOnPositioner(DescartesProblem<FloatType>& prob,
                              const Eigen::VectorXd& joint_waypoint,
                              const PlanInstruction& parent_instruction,
                              const std::vector<std::string> &active_links,
                              int index);

};
}

#endif // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
