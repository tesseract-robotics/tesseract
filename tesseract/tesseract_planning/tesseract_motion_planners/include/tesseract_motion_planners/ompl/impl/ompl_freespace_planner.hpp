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

#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_motion_planners
{
OMPLFreespacePlannerStatusCategory::OMPLFreespacePlannerStatusCategory(std::string name) : name_(name) {}
const std::string& OMPLFreespacePlannerStatusCategory::name() const noexcept { return name_; }
std::string OMPLFreespacePlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case IsConfigured:
    {
      return "Is Configured";
    }
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case ErrorIsNotConfigured:
    {
      return "Planner is not configured, must call setConfiguration prior to calling solve.";
    }
    case ErrorFailedToParseConfig:
    {
      return "Failed to parse config data";
    }
    case ErrorFailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

/** @brief Construct a basic planner */
template <typename PlannerType>
OMPLFreespacePlanner<PlannerType>::OMPLFreespacePlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const OMPLFreespacePlannerStatusCategory>(name))
{
}

template <typename PlannerType>
bool OMPLFreespacePlanner<PlannerType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template <typename PlannerType>
tesseract_common::StatusCode OMPLFreespacePlanner<PlannerType>::solve(PlannerResponse& response, const bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  tesseract_motion_planners::PlannerResponse planning_response;

  ompl::tools::OptimizePlan op(simple_setup_->getProblemDefinition());
  for (auto i = 0; i < config_->num_threads; ++i)
  {
    std::shared_ptr<PlannerType> planner = std::make_shared<PlannerType>(simple_setup_->getSpaceInformation());
    config_->settings.apply(*planner);
    op.addPlanner(planner);
  }
  // Solve problem. Results are stored in the response
  ompl::base::PlannerStatus status = op.solve(config_->planning_time, config_->max_solutions, config_->num_threads);

  if (!status || !simple_setup_->haveExactSolutionPath())
  {
    planning_response.status = tesseract_common::StatusCode(
        OMPLFreespacePlannerStatusCategory::ErrorFailedToFindValidSolution, status_category_);
    return planning_response.status;
  }

  if (config_->simplify)
    simple_setup_->simplifySolution();

  ompl::geometric::PathGeometric& path = simple_setup_->getSolutionPath();

  // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
  if (!config_->simplify && path.getStateCount() < config_->n_output_states)
    path.interpolate(config_->n_output_states);

  planning_response.status =
      tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::SolutionFound, status_category_);
  planning_response.joint_trajectory.trajectory = toTrajArray(path);
  planning_response.joint_trajectory.joint_names = kin_->getJointNames();

  response = std::move(planning_response);
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
                            config_->manipulator);
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
  ompl::base::RealVectorStateSpace* space = new ompl::base::RealVectorStateSpace();
  for (unsigned i = 0; i < dof; ++i)
    space->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

  space->setStateSamplerAllocator(
      std::bind(&OMPLFreespacePlanner::allocWeightedRealVectorStateSampler, this, std::placeholders::_1));

  ompl::base::StateSpacePtr state_space_ptr(space);
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
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = start_position->getPositions()[i];

  ompl::base::ScopedState<> goal_state(simple_setup_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = end_position->getPositions()[i];

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
        mv = std::make_shared<ContinuousMotionValidator>(simple_setup_->getSpaceInformation(), env, kin_);
        simple_setup_->getSpaceInformation()->setMotionValidator(std::move(mv));
      }
      else
      {
        mv = std::make_shared<DiscreteMotionValidator>(simple_setup_->getSpaceInformation(), env, kin_);
        simple_setup_->getSpaceInformation()->setMotionValidator(std::move(mv));
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
