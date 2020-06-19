#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_robot_positioner_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_external_positioner_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>

#include <descartes_light/interface/collision_interface.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/compound_edge_evaluator.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::Isometry3d& cartesian_waypoint,
                                                   const PlanInstruction& parent_instruction,
                                                   const std::vector<std::string> &active_links,
                                                   int index)
{
  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain) */
  auto it = std::find(active_links.begin(), active_links.end(), parent_instruction.getWorkingFrame());
  if (it != active_links.end() && prob.configuration != DescartesProblemD::Configuration::ROBOT_WITH_EXTERNAL_POSITIONER)
    throw std::runtime_error("DescartesDefaultPlanProfile: Assigned dynamic waypoint but configuration is not ROBOT_WITH_EXTERNAL_POSITIONER");

  if (prob.configuration == DescartesProblemD::Configuration::ROBOT_ONLY)
    applyRobotOnly(prob, cartesian_waypoint, parent_instruction, active_links, index);
  else if (prob.configuration == DescartesProblemD::Configuration::ROBOT_ON_POSITIONER)
    applyRobotOnPositioner(prob, cartesian_waypoint, parent_instruction, active_links, index);
  else if (prob.configuration == DescartesProblemD::Configuration::ROBOT_WITH_EXTERNAL_POSITIONER)
    applyRobotWithExternalPositioner(prob, cartesian_waypoint, parent_instruction, active_links, index);
  else
    throw std::runtime_error("DescartesDefaultPlanProfile: Unsupported configuration");

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(prob.dof));
        compound_evaluator->push_back(std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(), active_links, prob.joint_names, edge_collision_saftey_margin, edge_longest_valid_segment_length, allow_collision, debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(prob.dof));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(descartes_core::TimingConstraint<FloatType>(timing_constraint));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(&tesseract_kinematics::isWithinLimits<FloatType>, std::placeholders::_1, prob.joint_limits);
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::VectorXd& joint_waypoint,
                                                   const PlanInstruction& parent_instruction,
                                                   const std::vector<std::string> &active_links,
                                                   int index)
{
  if (prob.configuration == DescartesProblemD::Configuration::ROBOT_ONLY)
    applyRobotOnly(prob, joint_waypoint, parent_instruction, active_links, index);
  else if (prob.configuration == DescartesProblemD::Configuration::ROBOT_ON_POSITIONER)
    applyRobotOnPositioner(prob, joint_waypoint, parent_instruction, active_links, index);
  else if (prob.configuration == DescartesProblemD::Configuration::ROBOT_WITH_EXTERNAL_POSITIONER)
    applyRobotWithExternalPositioner(prob, joint_waypoint, parent_instruction, active_links, index);
  else
    throw std::runtime_error("DescartesDefaultPlanProfile: Unsupported configuration");

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(prob.dof));
        compound_evaluator->push_back(std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(), active_links, prob.joint_names, edge_collision_saftey_margin, edge_longest_valid_segment_length, allow_collision, debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(prob.dof));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(descartes_core::TimingConstraint<FloatType>(timing_constraint));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(&tesseract_kinematics::isWithinLimits<FloatType>, std::placeholders::_1, prob.joint_limits);
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotOnly(DescartesProblem<FloatType>& prob,
                                                            const Eigen::Isometry3d& cartesian_waypoint,
                                                            const PlanInstruction& parent_instruction,
                                                            const std::vector<std::string> &active_links,
                                                            int index)
{
  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironmentConst(), active_links, prob.manip_fwd_kin->getJointNames(), collision_safety_margin, debug);

  // Check if the waypoint is not relative to the world coordinate system
  Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
  if (!parent_instruction.getWorkingFrame().empty())
    world_to_waypoint = prob.env_state->link_transforms.at(parent_instruction.getWorkingFrame());

  auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(world_to_waypoint * cartesian_waypoint,
                                                                    target_pose_sampler,
                                                                    prob.manip_inv_kin,
                                                                    ci,
                                                                    prob.env_state,
                                                                    parent_instruction.getTCP(),
                                                                    prob.manip_reach,
                                                                    allow_collision,
                                                                    is_valid);
  prob.samplers.push_back(std::move(sampler));
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotOnly(DescartesProblem<FloatType>& prob,
                                                            const Eigen::VectorXd& joint_waypoint,
                                                            const PlanInstruction& parent_instruction,
                                                            const std::vector<std::string> &active_links,
                                                            int index)
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotOnPositioner(DescartesProblem<FloatType>& prob,
                                                                    const Eigen::Isometry3d& cartesian_waypoint,
                                                                    const PlanInstruction& parent_instruction,
                                                                    const std::vector<std::string> &active_links,
                                                                    int index)
{
  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironmentConst(), active_links, prob.manip_fwd_kin->getJointNames(), collision_safety_margin, debug);

  // Check if the waypoint is not relative to the world coordinate system
  Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
  if (!parent_instruction.getWorkingFrame().empty())
    world_to_waypoint = prob.env_state->link_transforms.at(parent_instruction.getWorkingFrame());

  auto sampler = std::make_shared<DescartesRobotPositionerSampler<FloatType>>(world_to_waypoint * cartesian_waypoint,
                                                                              target_pose_sampler,
                                                                              prob.positioner_fwd_kin,
                                                                              prob.manip_inv_kin,
                                                                              ci,
                                                                              prob.env_state,
                                                                              positioner_sample_resolution,
                                                                              parent_instruction.getTCP(),
                                                                              prob.manip_reach,
                                                                              allow_collision,
                                                                              is_valid);
  prob.samplers.push_back(std::move(sampler));
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotOnPositioner(DescartesProblem<FloatType>& prob,
                                                                    const Eigen::VectorXd& joint_waypoint,
                                                                    const PlanInstruction& parent_instruction,
                                                                    const std::vector<std::string> &active_links,
                                                                    int index)
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotWithExternalPositioner(DescartesProblem<FloatType>& prob,
                                                                              const Eigen::Isometry3d& cartesian_waypoint,
                                                                              const PlanInstruction& parent_instruction,
                                                                              const std::vector<std::string> &active_links,
                                                                              int index)
{
  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironmentConst(), active_links, prob.manip_fwd_kin->getJointNames(), collision_safety_margin, debug);

  // Check if the waypoint is not relative to the world coordinate system
  Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
  if (!parent_instruction.getWorkingFrame().empty())
    world_to_waypoint = prob.env_state->link_transforms.at(parent_instruction.getWorkingFrame());

  auto sampler = std::make_shared<DescartesExternalPositionerSampler<FloatType>>(world_to_waypoint * cartesian_waypoint,
                                                                                 target_pose_sampler,
                                                                                 prob.positioner_fwd_kin,
                                                                                 prob.manip_inv_kin,
                                                                                 ci,
                                                                                 prob.env_state,
                                                                                 positioner_sample_resolution,
                                                                                 parent_instruction.getTCP(),
                                                                                 prob.manip_reach,
                                                                                 allow_collision,
                                                                                 is_valid);
  prob.samplers.push_back(std::move(sampler));
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::applyRobotWithExternalPositioner(DescartesProblem<FloatType>& prob,
                                                                              const Eigen::VectorXd& joint_waypoint,
                                                                              const PlanInstruction& parent_instruction,
                                                                              const std::vector<std::string> &active_links,
                                                                              int index)
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));
}

}

#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
