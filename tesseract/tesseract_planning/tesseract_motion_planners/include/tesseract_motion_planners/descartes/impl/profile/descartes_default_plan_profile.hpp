#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
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
                                                   const std::vector<std::string>& active_links,
                                                   int index)
{
  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain) */
  auto it = std::find(active_links.begin(), active_links.end(), prob.manip_inv_kin->getBaseLinkName());
  if (it != active_links.end() && prob.manip_inv_kin->getBaseLinkName() != parent_instruction.getWorkingFrame())
    throw std::runtime_error("DescartesDefaultPlanProfile: Assigned dynamic waypoint but parent instruction working is "
                             "not set to the base link of manipulator!");

  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                         active_links,
                                                         prob.manip_inv_kin->getJointNames(),
                                                         collision_safety_margin,
                                                         debug);

  Eigen::Isometry3d manip_baselink_to_waypoint = Eigen::Isometry3d::Identity();
  if (it == active_links.end())
  {
    // Check if the waypoint is not relative to the manipulator base coordinate system
    Eigen::Isometry3d world_to_waypoint = cartesian_waypoint;
    if (!parent_instruction.getWorkingFrame().empty())
      world_to_waypoint = prob.env_state->link_transforms.at(parent_instruction.getWorkingFrame()) * cartesian_waypoint;

    Eigen::Isometry3d world_to_base_link = prob.env_state->link_transforms.at(prob.manip_inv_kin->getBaseLinkName());
    manip_baselink_to_waypoint = world_to_base_link.inverse() * world_to_waypoint;
  }

  auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(manip_baselink_to_waypoint,
                                                                    target_pose_sampler,
                                                                    prob.manip_inv_kin,
                                                                    ci,
                                                                    parent_instruction.getTCP(),
                                                                    allow_collision,
                                                                    is_valid);
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
        compound_evaluator->push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_saftey_margin,
                                                                         edge_longest_valid_segment_length,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(
      descartes_core::TimingConstraint<FloatType>(static_cast<FloatType>(timing_constraint)));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(
        &tesseract_kinematics::isWithinLimits<FloatType>, std::placeholders::_1, prob.manip_inv_kin->getLimits());

  prob.num_threads = num_threads;
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::VectorXd& joint_waypoint,
                                                   const PlanInstruction& /*parent_instruction*/,
                                                   const std::vector<std::string>& active_links,
                                                   int index)
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
        compound_evaluator->push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_saftey_margin,
                                                                         edge_longest_valid_segment_length,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(
      descartes_core::TimingConstraint<FloatType>(static_cast<FloatType>(timing_constraint)));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(
        &tesseract_kinematics::isWithinLimits<FloatType>, std::placeholders::_1, prob.manip_inv_kin->getLimits());

  prob.num_threads = num_threads;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
