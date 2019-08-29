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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

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
    case IsNotConfigured:
    {
      return "Planner is not configured, must call setConfiguration prior to calling solve.";
    }
    case FailedToParseConfig:
    {
      return "Failed to parse config data";
    }
    case FailedToFindValidSolution:
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
tesseract_common::StatusCode OMPLFreespacePlanner<PlannerType>::solve(PlannerResponse& response)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  tesseract_motion_planners::PlannerResponse planning_response;

  // Solve problem. Results are stored in the response
  ompl::base::PlannerStatus status = simple_setup_->solve(config_->planning_time);

  if (!status)
  {
    planning_response.status =
        tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::FailedToFindValidSolution, status_category_);
    return planning_response.status;
  }

  if (config_->simplify)
    simple_setup_->simplifySolution();

  ompl::geometric::PathGeometric& path = simple_setup_->getSolutionPath();

  planning_response.status =
      tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::SolutionFound, status_category_);
  planning_response.joint_trajectory.trajectory = toTrajArray(path);
  planning_response.joint_trajectory.joint_names = kin_->getJointNames();

  response = std::move(planning_response);
  return planning_response.status;
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
    return tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::IsNotConfigured, status_category_);

  return tesseract_common::StatusCode(OMPLFreespacePlannerStatusCategory::IsConfigured, status_category_);
}

template <typename PlannerType>
bool OMPLFreespacePlanner<PlannerType>::setConfiguration(const OMPLFreespacePlannerConfig& config)
{
  // Check that parameters are valid
  if (config.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In ompl_freespace_planner: tesseract is a required parameter and has not been set");
    return false;
  }

  kin_ = config.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config.manipulator);
  if (kin_ == nullptr)
  {
    CONSOLE_BRIDGE_logError("In ompl_freespace_planner: failed to get kinematics object for manipulator: %s.",
                            config.manipulator);
    return false;
  }

  const tesseract_environment::Environment::ConstPtr& env = config.tesseract->getEnvironmentConst();
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
  {
    space->addDimension(joint_names[i], limits(i, 0), limits(i, 1));
  }

  ompl::base::StateSpacePtr state_space_ptr(space);
  state_space_ptr->setLongestValidSegmentFraction(config.longest_valid_segment_fraction);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);

  // Setup state checking functionality
  if (config.svc == nullptr && config.collision_check)
    simple_setup_->setStateValidityChecker(std::bind(&OMPLFreespacePlanner::isStateValid, this, std::placeholders::_1));
  else if (config.svc != nullptr)
    simple_setup_->setStateValidityChecker(config.svc);

  if (config.collision_check && config.collision_continuous && config.mv == nullptr)
  {
    ompl::base::MotionValidatorPtr mv =
        std::make_shared<ContinuousMotionValidator>(simple_setup_->getSpaceInformation(), env, kin_);
    simple_setup_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }
  else if (config.mv != nullptr)
  {
    simple_setup_->getSpaceInformation()->setMotionValidator(config.mv);
  }

  JointWaypoint::Ptr start_position;
  JointWaypoint::Ptr end_position;

  // Set initial point
  auto start_type = config.start_waypoint->getType();
  switch (start_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      start_position = std::static_pointer_cast<JointWaypoint>(config.start_waypoint);
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In ompl_freespace_planner: only support joint waypoints for start_waypoint");
      return false;
    }
  };

  // Set end point
  auto end_type = config.end_waypoint->getType();
  switch (end_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      end_position = std::static_pointer_cast<JointWaypoint>(config.end_waypoint);
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In ompl_freespace_planner: only support joint waypoints for end_waypoint");
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

  // Set the ompl planner
  ompl::base::PlannerPtr planner = std::make_shared<PlannerType>(simple_setup_->getSpaceInformation());
  simple_setup_->setPlanner(planner);

  discrete_contact_manager_ = env->getDiscreteContactManager();
  discrete_contact_manager_->setActiveCollisionObjects(adj_map_->getActiveLinkNames());
  discrete_contact_manager_->setContactDistanceThreshold(config.collision_safety_margin);

  continuous_contact_manager_ = env->getContinuousContactManager();
  continuous_contact_manager_->setActiveCollisionObjects(adj_map_->getActiveLinkNames());
  continuous_contact_manager_->setContactDistanceThreshold(config.collision_safety_margin);

  config_ = std::make_shared<OMPLFreespacePlannerConfig>(config);

  return true;
}

template <typename PlannerType>
bool OMPLFreespacePlanner<PlannerType>::isStateValid(const ompl::base::State* state) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = kin_->numJoints();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state =
      config_->tesseract->getEnvironmentConst()->getState(kin_->getJointNames(), joint_angles);

  // Need to get thread id
  tesseract_collision::DiscreteContactManager::Ptr cm = discrete_contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_HPP
