/**
 * @file ompl_freespace_planner.hpp
 * @brief Tesseract OMPL planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_HPP
#define TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_motion_planners
{
/** @brief Construct a basic planner */
template <typename PlannerType>
OMPLFreespacePlanner<PlannerType>::OMPLFreespacePlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const OMPLFreespacePlannerStatusCategory>(name_))
{
}

template <typename PlannerType>
bool OMPLFreespacePlanner<PlannerType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template <typename PlannerType>
tesseract_common::StatusCode OMPLFreespacePlanner<PlannerType>::solve(PlannerResponse& response, bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  ompl::tools::OptimizePlan op(simple_setup_->getProblemDefinition());
  for (auto i = 0; i < config_->num_threads; ++i)
  {
    std::shared_ptr<PlannerType> planner = std::make_shared<PlannerType>(simple_setup_->getSpaceInformation());
    config_->settings.apply(*planner);
    op.addPlanner(planner);
  }
  // Solve problem. Results are stored in the response
  ompl::base::PlannerStatus status = op.solve(config_->planning_time,
                                              static_cast<unsigned>(config_->max_solutions),
                                              static_cast<unsigned>(config_->num_threads));

  if (!status || !simple_setup_->haveExactSolutionPath())
  {
    response.status = tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::ErrorFailedToFindValidSolution,
                                                   status_category_);
    return response.status;
  }

  if (config_->simplify)
  {
    simple_setup_->simplifySolution();
  }
  else
  {
    // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
    auto num_output_states = static_cast<unsigned>(config_->n_output_states);
    if (simple_setup_->getSolutionPath().getStateCount() < num_output_states)
    {
      simple_setup_->getSolutionPath().interpolate(num_output_states);
    }
    else
    {
      // Now try to simplify the trajectory to get it under the requested number of output states
      // The interpolate function only executes if the current number of states is less than the requested
      simple_setup_->simplifySolution();
      if (simple_setup_->getSolutionPath().getStateCount() < num_output_states)
        simple_setup_->getSolutionPath().interpolate(num_output_states);
    }
  }

  tesseract_common::TrajArray traj = toTrajArray(simple_setup_->getSolutionPath());

  // Check and report collisions
  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = config_->tesseract->getEnvironmentConst()->getStateSolver();
  continuous_contact_manager_->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = tesseract_environment::checkTrajectory(collisions,
                                                      *continuous_contact_manager_,
                                                      *state_solver,
                                                      kin_->getJointNames(),
                                                      traj,
                                                      config_->longest_valid_segment_length,
                                                      tesseract_collision::ContactTestType::FIRST,
                                                      verbose);

  // Set the contact distance back to original incase solve was called again.
  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  // Send response
  response.joint_trajectory.trajectory = traj;
  response.joint_trajectory.joint_names = kin_->getJointNames();
  if (found)
  {
    response.status = tesseract_common::StatusCode(
        OMPLFreespacePlannerStatusCategory::ErrorFoundValidSolutionInCollision, status_category_);
  }
  else
  {
    response.status = tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::SolutionFound, status_category_);
    CONSOLE_BRIDGE_logInform("%s, final trajectory is collision free", name_.c_str());
  }

  return response.status;
}

template <typename PlannerType>
void OMPLFreespacePlanner<PlannerType>::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
  kin_ = nullptr;
  adj_map_ = nullptr;
  discrete_contact_manager_ = nullptr;
  continuous_contact_manager_ = nullptr;
  simple_setup_ = nullptr;
}

template <typename PlannerType>
tesseract_common::StatusCode OMPLFreespacePlanner<PlannerType>::isConfigured() const
{
  if (config_ == nullptr)
    return tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::ErrorIsNotConfigured, status_category_);

  return tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::IsConfigured, status_category_);
}

template <typename PlannerType>
bool OMPLFreespacePlanner<PlannerType>::setConfiguration(const OMPLFreespacePlannerConfig<PlannerType>& config)
{
  config_ = std::make_shared<OMPLFreespacePlannerConfig<PlannerType>>(config);

  // Check that parameters are valid
  if (config_->tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In ompl_freespace_planner: tesseract is a required parameter and has not been set");
    config_ = nullptr;
    return false;
  }

  kin_ = config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_->manipulator);
  if (kin_ == nullptr)
  {
    CONSOLE_BRIDGE_logError("In ompl_freespace_planner: failed to get kinematics object for manipulator: %s.",
                            config_->manipulator.c_str());
    config_ = nullptr;
    return false;
  }

  if (config_->weights.size() == 0)
  {
    config_->weights = Eigen::VectorXd::Ones(kin_->numJoints());
  }
  else if (config_->weights.size() != kin_->numJoints())
  {
    CONSOLE_BRIDGE_logError("In ompl_freespace_planner: The weights must be the same length as the number of joints or "
                            "have a length of zero!");
    config_ = nullptr;
    return false;
  }

  const tesseract_environment::Environment::ConstPtr& env = config_->tesseract->getEnvironmentConst();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  adj_map_ = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin_->getActiveLinkNames(), env->getCurrentState()->transforms);

  const std::vector<std::string>& joint_names = kin_->getJointNames();
  const auto dof = kin_->numJoints();
  const auto& limits = kin_->getLimits();

  // Construct the OMPL state space for this manipulator
  auto* space = new ompl::base::RealVectorStateSpace();
  for (unsigned i = 0; i < dof; ++i)
    space->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

  space->setStateSamplerAllocator(
      std::bind(&OMPLFreespacePlanner::allocWeightedRealVectorStateSampler, this, std::placeholders::_1));

  ompl::base::StateSpacePtr state_space_ptr(space);
  if (config_->longest_valid_segment_fraction > 0 && config_->longest_valid_segment_length > 0)
  {
    double val = std::min(config_->longest_valid_segment_fraction,
                          config_->longest_valid_segment_length / state_space_ptr->getMaximumExtent());
    config_->longest_valid_segment_fraction = val;
    config_->longest_valid_segment_length = val * state_space_ptr->getMaximumExtent();
    state_space_ptr->setLongestValidSegmentFraction(val);
  }
  else if (config_->longest_valid_segment_fraction > 0)
  {
    config_->longest_valid_segment_length =
        config_->longest_valid_segment_fraction * state_space_ptr->getMaximumExtent();
  }
  else if (config_->longest_valid_segment_length > 0)
  {
    config_->longest_valid_segment_fraction =
        config_->longest_valid_segment_length / state_space_ptr->getMaximumExtent();
  }
  else
  {
    config_->longest_valid_segment_fraction = 0.01;
    config_->longest_valid_segment_length = 0.01 * state_space_ptr->getMaximumExtent();
  }
  state_space_ptr->setLongestValidSegmentFraction(config_->longest_valid_segment_fraction);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);

  JointWaypoint::Ptr start_position;
  JointWaypoint::Ptr end_position;

  // Get descrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::DiscreteContactManager::Ptr cm = env->getDiscreteContactManager();
  cm->setActiveCollisionObjects(adj_map_->getActiveLinkNames());
  cm->setContactDistanceThreshold(config_->collision_safety_margin);

  // Set initial point
  auto start_type = config_->start_waypoint->getType();
  switch (start_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      start_position = std::static_pointer_cast<JointWaypoint>(config_->start_waypoint);
      tesseract_environment::EnvState::Ptr s =
          env->getState(start_position->getNames(), start_position->getPositions());

      for (const auto& link_name : adj_map_->getActiveLinkNames())
        cm->setCollisionObjectsTransform(link_name, s->transforms[link_name]);

      tesseract_collision::ContactResultMap contact_map;
      cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

      if (!contact_map.empty())
      {
        CONSOLE_BRIDGE_logError("In ompl_freespace_planner: Start state is in collision");
        config_ = nullptr;
        return false;
      }
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In ompl_freespace_planner: only support joint waypoints for start_waypoint");
      config_ = nullptr;
      return false;
    }
  };

  // Set end point
  auto end_type = config_->end_waypoint->getType();
  switch (end_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      end_position = std::static_pointer_cast<JointWaypoint>(config_->end_waypoint);
      tesseract_environment::EnvState::Ptr s = env->getState(end_position->getNames(), end_position->getPositions());

      for (const auto& link_name : adj_map_->getActiveLinkNames())
        cm->setCollisionObjectsTransform(link_name, s->transforms[link_name]);

      tesseract_collision::ContactResultMap contact_map;
      cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

      if (!contact_map.empty())
      {
        CONSOLE_BRIDGE_logError("In ompl_freespace_planner: End state is in collision");
        config_ = nullptr;
        return false;
      }
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In ompl_freespace_planner: only support joint waypoints for end_waypoint");
      config_ = nullptr;
      return false;
    }
  };

  ompl::base::ScopedState<> start_state(simple_setup_->getStateSpace());
  const Eigen::VectorXd sp = start_position->getPositions(kin_->getJointNames());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = sp[i];

  ompl::base::ScopedState<> goal_state(simple_setup_->getStateSpace());
  const Eigen::VectorXd ep = end_position->getPositions(kin_->getJointNames());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = ep[i];

  simple_setup_->setStartAndGoalStates(start_state, goal_state);

  // Setup state checking functionality
  if (config_->svc != nullptr)
    simple_setup_->setStateValidityChecker(config_->svc);

  // Setup motion validation (i.e. collision checking)
  if (config_->mv != nullptr)
  {
    simple_setup_->getSpaceInformation()->setMotionValidator(config_->mv);
  }
  else
  {
    if (config_->collision_check)
    {
      ompl::base::MotionValidatorPtr mv;
      if (config_->collision_continuous)
      {
        mv = std::make_shared<ContinuousMotionValidator>(
            simple_setup_->getSpaceInformation(), env, kin_, config_->collision_safety_margin);
        simple_setup_->getSpaceInformation()->setMotionValidator(mv);
      }
      else
      {
        mv = std::make_shared<DiscreteMotionValidator>(
            simple_setup_->getSpaceInformation(), env, kin_, config_->collision_safety_margin);
        simple_setup_->getSpaceInformation()->setMotionValidator(mv);
      }
    }
  }

  // make sure the planners run until the time limit, and get the best possible solution
  simple_setup_->getProblemDefinition()->setOptimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(simple_setup_->getSpaceInformation()));

  discrete_contact_manager_ = env->getDiscreteContactManager();
  discrete_contact_manager_->setActiveCollisionObjects(adj_map_->getActiveLinkNames());
  discrete_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  continuous_contact_manager_ = env->getContinuousContactManager();
  continuous_contact_manager_->setActiveCollisionObjects(adj_map_->getActiveLinkNames());
  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  return true;
}

template <typename PlannerType>
ompl::base::StateSamplerPtr
OMPLFreespacePlanner<PlannerType>::allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const
{
  return std::make_shared<WeightedRealVectorStateSampler>(space, config_->weights);
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_HPP
