/**
 * @file ompl_default_plan_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalStates.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/utils.h>

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>
#include <tesseract_motion_planners/ompl/compound_state_validator.h>

namespace tesseract_planning
{
void OMPLDefaultPlanProfile::setup(OMPLProblem& prob)
{
  prob.planners = planners;
  prob.max_solutions = max_solutions;
  prob.simplify = simplify;
  prob.optimize = optimize;
  prob.contact_checker->setContactDistanceThreshold(collision_safety_margin);

  const tesseract_environment::Environment::ConstPtr& env = prob.tesseract->getEnvironmentConst();
  const std::vector<std::string>& joint_names = prob.manip_fwd_kin->getJointNames();
  const auto dof = prob.manip_fwd_kin->numJoints();
  const auto& limits = prob.manip_fwd_kin->getLimits().joint_limits;

  if (state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
    prob.extractor = std::bind(
        &tesseract_planning::RealVectorStateSpaceExtractor, std::placeholders::_1, prob.manip_inv_kin->numJoints());
#ifndef OMPL_LESS_1_4_0
  else if (state_space == OMPLProblemStateSpace::REAL_CONSTRAINTED_STATE_SPACE)
    prob.extractor = tesseract_planning::ConstrainedStateSpaceExtractor;
#endif
  else
    throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Unsupported configuration!");

  if (weights.size() == 1)
    weights = Eigen::VectorXd::Constant(1, dof, weights(0));
  else if (weights.size() != dof)
    weights = Eigen::VectorXd::Ones(dof);

  if (prob.state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
  {
    // Construct the OMPL state space for this manipulator
    ompl::base::StateSpacePtr state_space_ptr;

    auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
    for (unsigned i = 0; i < dof; ++i)
      rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

    if (state_sampler_allocator)
    {
      rss->setStateSamplerAllocator(state_sampler_allocator);
    }
    else
    {
      rss->setStateSamplerAllocator(
          std::bind(&OMPLDefaultPlanProfile::allocWeightedRealVectorStateSampler, this, std::placeholders::_1, limits));
    }

    state_space_ptr = rss;

    // Setup Longest Valid Segment
    processLongestValidSegment(state_space_ptr, longest_valid_segment_fraction, longest_valid_segment_length);

    // Create Simple Setup from state space
    prob.simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);

    // Setup state checking functionality
    ompl::base::StateValidityCheckerPtr svc_without_collision = processStateValidator(prob, env, prob.manip_fwd_kin);

    // Setup motion validation (i.e. collision checking)
    processMotionValidator(svc_without_collision, prob, env, prob.manip_fwd_kin);

    // make sure the planners run until the time limit, and get the best possible solution
    processOptimizationObjective(prob);
  }
}

void OMPLDefaultPlanProfile::applyGoalStates(OMPLProblem& prob,
                                             const Eigen::Isometry3d& cartesian_waypoint,
                                             const Instruction& parent_instruction,
                                             const ManipulatorInfo& manip_info,
                                             const std::vector<std::string>& /*active_links*/,
                                             int /*index*/)
{
  const auto dof = prob.manip_fwd_kin->numJoints();
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.isEmpty() && base_instruction->getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction->getManipulatorInfo().isEmpty()) ? manip_info : base_instruction->getManipulatorInfo();

  // Check if the waypoint is not relative to the manipulator base coordinate system and at tool0
  Eigen::Isometry3d world_to_waypoint = cartesian_waypoint;
  if (!mi.working_frame.empty())
    world_to_waypoint = prob.env_state->link_transforms.at(mi.working_frame) * cartesian_waypoint;

  Eigen::Isometry3d world_to_base_link = prob.env_state->link_transforms.at(prob.manip_inv_kin->getBaseLinkName());
  Eigen::Isometry3d manip_baselink_to_waypoint = world_to_base_link.inverse() * world_to_waypoint;
  Eigen::Isometry3d manip_baselink_to_tool0 = manip_baselink_to_waypoint * mi.tcp.inverse();

  if (prob.state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
  {
    /** @todo Need to add descartes pose sample to ompl profile */
    Eigen::VectorXd joint_solutions;
    prob.manip_inv_kin->calcInvKin(joint_solutions, manip_baselink_to_tool0, Eigen::VectorXd::Zero(dof));
    long num_solutions = joint_solutions.size() / dof;
    auto goal_states = std::make_shared<ompl::base::GoalStates>(prob.simple_setup->getSpaceInformation());
    for (long i = 0; i < num_solutions; ++i)
    {
      auto solution = joint_solutions.middleRows(i * dof, dof);
      // Get descrete contact manager for testing provided start and end position
      // This is required because collision checking happens in motion validators now
      // instead of the isValid function to avoid unnecessary collision checks.
      if (!checkStateInCollision(prob, solution))
      {
        ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
        for (unsigned i = 0; i < dof; ++i)
          goal_state[i] = solution(i);

        goal_states->addState(goal_state);
      }
    }

    if (!goal_states->hasStates())
      throw std::runtime_error("In OMPLPlannerFreespaceConfig: All goal states are in collision");

    prob.simple_setup->setGoal(goal_states);
  }
}

void OMPLDefaultPlanProfile::applyGoalStates(OMPLProblem& prob,
                                             const Eigen::VectorXd& joint_waypoint,
                                             const Instruction& /*parent_instruction*/,
                                             const ManipulatorInfo& /*manip_info*/,
                                             const std::vector<std::string>& /*active_links*/,
                                             int /*index*/)
{
  const auto dof = prob.manip_fwd_kin->numJoints();

  if (prob.state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
  {
    // Get descrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (checkStateInCollision(prob, joint_waypoint))
    {
      CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
    }

    ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
    for (unsigned i = 0; i < dof; ++i)
      goal_state[i] = joint_waypoint[i];

    prob.simple_setup->setGoalState(goal_state);
  }
}

void OMPLDefaultPlanProfile::applyStartStates(OMPLProblem& prob,
                                              const Eigen::Isometry3d& cartesian_waypoint,
                                              const Instruction& parent_instruction,
                                              const ManipulatorInfo& manip_info,
                                              const std::vector<std::string>& /*active_links*/,
                                              int /*index*/)
{
  const auto dof = prob.manip_fwd_kin->numJoints();
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.isEmpty() && base_instruction->getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction->getManipulatorInfo().isEmpty()) ? manip_info : base_instruction->getManipulatorInfo();

  // Check if the waypoint is not relative to the manipulator base coordinate system and at tool0
  Eigen::Isometry3d world_to_waypoint = cartesian_waypoint;
  if (!mi.working_frame.empty())
    world_to_waypoint = prob.env_state->link_transforms.at(mi.working_frame) * cartesian_waypoint;

  Eigen::Isometry3d world_to_base_link = prob.env_state->link_transforms.at(prob.manip_inv_kin->getBaseLinkName());
  Eigen::Isometry3d manip_baselink_to_waypoint = world_to_base_link.inverse() * world_to_waypoint;
  Eigen::Isometry3d manip_baselink_to_tool0 = manip_baselink_to_waypoint * mi.tcp.inverse();

  if (prob.state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
  {
    /** @todo Need to add descartes pose sampler to ompl profile */
    Eigen::VectorXd joint_solutions;

    /** @todo Need to also provide the seed instruction to use here */
    prob.manip_inv_kin->calcInvKin(joint_solutions, manip_baselink_to_tool0, Eigen::VectorXd::Zero(dof));
    long num_solutions = joint_solutions.size() / dof;
    bool found_start_state = false;
    for (long i = 0; i < num_solutions; ++i)
    {
      auto solution = joint_solutions.middleRows(i * dof, dof);
      // Get descrete contact manager for testing provided start and end position
      // This is required because collision checking happens in motion validators now
      // instead of the isValid function to avoid unnecessary collision checks.
      if (!checkStateInCollision(prob, solution))
      {
        found_start_state = true;
        ompl::base::ScopedState<> start_state(prob.simple_setup->getStateSpace());
        for (unsigned i = 0; i < dof; ++i)
          start_state[i] = solution(i);

        prob.simple_setup->addStartState(start_state);
      }
    }

    if (!found_start_state)
      throw std::runtime_error("In OMPLPlannerFreespaceConfig: All start states are in collision");
  }
}

void OMPLDefaultPlanProfile::applyStartStates(OMPLProblem& prob,
                                              const Eigen::VectorXd& joint_waypoint,
                                              const Instruction& /*parent_instruction*/,
                                              const ManipulatorInfo& /*manip_info*/,
                                              const std::vector<std::string>& /*active_links*/,
                                              int /*index*/)
{
  const auto dof = prob.manip_fwd_kin->numJoints();

  if (prob.state_space == OMPLProblemStateSpace::REAL_STATE_SPACE)
  {
    // Get descrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (checkStateInCollision(prob, joint_waypoint))
    {
      CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
    }

    ompl::base::ScopedState<> start_state(prob.simple_setup->getStateSpace());
    for (unsigned i = 0; i < dof; ++i)
      start_state[i] = joint_waypoint[i];

    prob.simple_setup->addStartState(start_state);
  }
}

ompl::base::StateValidityCheckerPtr
OMPLDefaultPlanProfile::processStateValidator(OMPLProblem& prob,
                                              const tesseract_environment::Environment::ConstPtr& env,
                                              const tesseract_kinematics::ForwardKinematics::ConstPtr& kin)
{
  ompl::base::StateValidityCheckerPtr svc_without_collision;
  auto csvc = std::make_shared<CompoundStateValidator>();
  if (svc_allocator != nullptr)
  {
    svc_without_collision = svc_allocator(prob.simple_setup->getSpaceInformation(), prob);
    csvc->addStateValidator(svc_without_collision);
  }

  if (collision_check && !collision_continuous)
  {
    auto svc = std::make_shared<StateCollisionValidator>(
        prob.simple_setup->getSpaceInformation(), env, kin, collision_safety_margin, prob.extractor);
    csvc->addStateValidator(svc);
  }
  prob.simple_setup->setStateValidityChecker(csvc);

  return svc_without_collision;
}

void OMPLDefaultPlanProfile::processMotionValidator(ompl::base::StateValidityCheckerPtr svc_without_collision,
                                                    OMPLProblem& prob,
                                                    const tesseract_environment::Environment::ConstPtr& env,
                                                    const tesseract_kinematics::ForwardKinematics::ConstPtr& kin)
{
  if (mv_allocator != nullptr)
  {
    auto mv = mv_allocator(prob.simple_setup->getSpaceInformation(), prob);
    prob.simple_setup->getSpaceInformation()->setMotionValidator(mv);
  }
  else
  {
    if (collision_check)
    {
      ompl::base::MotionValidatorPtr mv;
      if (collision_continuous)
      {
        mv = std::make_shared<ContinuousMotionValidator>(prob.simple_setup->getSpaceInformation(),
                                                         svc_without_collision,
                                                         env,
                                                         kin,
                                                         collision_safety_margin,
                                                         prob.extractor);
      }
      else
      {
        // Collision checking is preformed using the state validator which this calles.
        mv = std::make_shared<DiscreteMotionValidator>(prob.simple_setup->getSpaceInformation());
      }
      prob.simple_setup->getSpaceInformation()->setMotionValidator(mv);
    }
  }
}

void OMPLDefaultPlanProfile::processOptimizationObjective(OMPLProblem& prob)
{
  if (optimization_objective_allocator)
  {
    prob.simple_setup->getProblemDefinition()->setOptimizationObjective(
        optimization_objective_allocator(prob.simple_setup->getSpaceInformation(), prob));
  }
}

ompl::base::StateSamplerPtr
OMPLDefaultPlanProfile::allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                            const Eigen::MatrixX2d& limits) const
{
  return std::make_shared<WeightedRealVectorStateSampler>(space, weights, limits);
}

}  // namespace tesseract_planning
