#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DEFAULT_PLAN_PROFILE_HPP

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_robot_positioner_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>

#include <descartes_light/interface/collision_interface.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

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
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), parent_instruction.getWorkingFrame());
  if (it != active_links.end())
  {
//    ti = createDynamicCartesianWaypointTermInfo(cartesian_waypoint, index, parent_instruction.getWorkingFrame(), parent_instruction.getTCP(), cartesian_coeff, pci.kin->getTipLinkName(), term_type);
//    pci.cnt_infos.push_back(ti);
  }
  else
  {
//    ti = createCartesianWaypointTermInfo(cartesian_waypoint, index, parent_instruction.getWorkingFrame(), parent_instruction.getTCP(), cartesian_coeff, pci.kin->getTipLinkName(), term_type);
//    pci.cnt_infos.push_back(ti);
  }

}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::VectorXd& joint_waypoint,
                                                   const PlanInstruction& parent_instruction,
                                                   const std::vector<std::string> &active_links,
                                                   int index)
{

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

}

#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
