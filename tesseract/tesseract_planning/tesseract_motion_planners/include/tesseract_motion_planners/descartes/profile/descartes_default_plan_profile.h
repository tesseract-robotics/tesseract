#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/types.h>

#include <descartes_light/interface/edge_evaluator.h>
#include <Eigen/Geometry>

namespace tesseract_planning
{

template <typename FloatType>
class DescartesDefaultPlanProfile : public DescartesPlanProfile<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesDefaultPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesDefaultPlanProfile<FloatType>>;

  PoseSamplerFn target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) { return tesseract_common::VectorIsometry3d({ tool_pose }); };
  DescartesEdgeEvaluatorAllocatorFn<FloatType> edge_evaluator {nullptr};
  double timing_constraint = std::numeric_limits<FloatType>::max();
  Eigen::VectorXd positioner_sample_resolution = Eigen::VectorXd::Constant(1, 1, 0.01); // TODO Does this belong here or in the problem?

  // Applied to sampled states
  bool enable_collision {true};
  double collision_safety_margin {0};

  // Applied during edge evaluation
  bool enable_edge_collision {false};
  double edge_collision_saftey_margin {0};
  double edge_longest_valid_segment_length = 0.5;

  bool allow_collision {false};
  DescartesIsValidFn<FloatType> is_valid; // If not provided it adds a joint limit is valid function
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

  void applyRobotWithExternalPositioner(DescartesProblem<FloatType>& prob,
                                        const Eigen::Isometry3d& cartesian_waypoint,
                                        const PlanInstruction& parent_instruction,
                                        const std::vector<std::string> &active_links,
                                        int index);

  void applyRobotWithExternalPositioner(DescartesProblem<FloatType>& prob,
                                        const Eigen::VectorXd& joint_waypoint,
                                        const PlanInstruction& parent_instruction,
                                        const std::vector<std::string> &active_links,
                                        int index);

};

using DescartesDefaultPlanProfileF = DescartesDefaultPlanProfile<float>;
using DescartesDefaultPlanProfileD = DescartesDefaultPlanProfile<double>;
}

#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
