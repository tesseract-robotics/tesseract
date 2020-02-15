/**
 * @file ompl_planner_freespace_config.cpp
 * @brief Tesseract OMPL planner freespace config implementation.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
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

#ifndef OMPL_LESS_1_4_0
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>

namespace tesseract_motion_planners
{
/** @brief Construct a basic planner */
OMPLPlannerFreespaceConfig::OMPLPlannerFreespaceConfig(tesseract::Tesseract::ConstPtr tesseract,
                                                       std::string manipulator,
                                                       std::vector<OMPLPlannerConfigurator::ConstPtr> planners)
  : OMPLPlannerConfig(std::move(tesseract), std::move(manipulator), std::move(planners))
{
}

bool OMPLPlannerFreespaceConfig::generate()
{
  // Check that parameters are valid
  if (tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: tesseract is a required parameter and has not been set");
    return false;
  }

  tesseract_kinematics::ForwardKinematics::Ptr kin =
      tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: failed to get kinematics object for manipulator: %s.",
                            manipulator.c_str());
    return false;
  }

  if (weights.size() == 0)
  {
    weights = Eigen::VectorXd::Ones(kin->numJoints());
  }
  else if (weights.size() != kin->numJoints())
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: The weights must be the same length as the number of "
                            "joints or "
                            "have a length of zero!");
    return false;
  }

  const tesseract_environment::Environment::ConstPtr& env = tesseract->getEnvironmentConst();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->transforms);

  const std::vector<std::string>& joint_names = kin->getJointNames();
  const auto dof = kin->numJoints();
  const auto& limits = kin->getLimits();

  // Construct the OMPL state space for this manipulator
  ompl::base::StateSpacePtr state_space_ptr;

  auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
  for (unsigned i = 0; i < dof; ++i)
    rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

  if (state_sampler_allocator)
    rss->setStateSamplerAllocator(state_sampler_allocator);

#ifndef OMPL_LESS_1_4_0
  if (constraint)
    state_space_ptr = std::make_shared<ompl::base::ProjectedStateSpace>(rss, constraint);
  else
    state_space_ptr = rss;
#else
  state_space_ptr = rss;
#endif

  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
  {
    double val =
        std::min(longest_valid_segment_fraction, longest_valid_segment_length / state_space_ptr->getMaximumExtent());
    longest_valid_segment_fraction = val;
    longest_valid_segment_length = val * state_space_ptr->getMaximumExtent();
    state_space_ptr->setLongestValidSegmentFraction(val);
  }
  else if (longest_valid_segment_fraction > 0)
  {
    longest_valid_segment_length = longest_valid_segment_fraction * state_space_ptr->getMaximumExtent();
  }
  else if (longest_valid_segment_length > 0)
  {
    longest_valid_segment_fraction = longest_valid_segment_length / state_space_ptr->getMaximumExtent();
  }
  else
  {
    longest_valid_segment_fraction = 0.01;
    longest_valid_segment_length = 0.01 * state_space_ptr->getMaximumExtent();
  }
  state_space_ptr->setLongestValidSegmentFraction(longest_valid_segment_fraction);

#ifndef OMPL_LESS_1_4_0
  if (constraint)
  {
    auto csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(state_space_ptr);
    this->simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(csi);
  }
  else
  {
    this->simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);
  }
#else
  this->simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);
#endif

  JointWaypoint::Ptr start_position;
  JointWaypoint::Ptr end_position;

  // Get descrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::DiscreteContactManager::Ptr cm = env->getDiscreteContactManager();
  cm->setActiveCollisionObjects(adj_map->getActiveLinkNames());
  cm->setContactDistanceThreshold(collision_safety_margin);

  // Set initial point
  auto start_type = start_waypoint->getType();
  switch (start_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      start_position = std::static_pointer_cast<JointWaypoint>(start_waypoint);
      tesseract_environment::EnvState::Ptr s =
          env->getState(start_position->getNames(), start_position->getPositions());

      for (const auto& link_name : adj_map->getActiveLinkNames())
        cm->setCollisionObjectsTransform(link_name, s->transforms[link_name]);

      tesseract_collision::ContactResultMap contact_map;
      cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

      if (!contact_map.empty())
      {
        CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
        return false;
      }
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: only support joint waypoints for start_waypoint");
      return false;
    }
  };

  // Set end point
  auto end_type = end_waypoint->getType();
  switch (end_type)
  {
    case tesseract_motion_planners::WaypointType::JOINT_WAYPOINT:
    {
      end_position = std::static_pointer_cast<JointWaypoint>(end_waypoint);
      tesseract_environment::EnvState::Ptr s = env->getState(end_position->getNames(), end_position->getPositions());

      for (const auto& link_name : adj_map->getActiveLinkNames())
        cm->setCollisionObjectsTransform(link_name, s->transforms[link_name]);

      tesseract_collision::ContactResultMap contact_map;
      cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

      if (!contact_map.empty())
      {
        CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: End state is in collision");
        return false;
      }
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: only support joint waypoints for end_waypoint");
      return false;
    }
  };

  ompl::base::ScopedState<> start_state(simple_setup->getStateSpace());
  const Eigen::VectorXd sp = start_position->getPositions(kin->getJointNames());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = sp[i];

  ompl::base::ScopedState<> goal_state(simple_setup->getStateSpace());
  const Eigen::VectorXd ep = end_position->getPositions(kin->getJointNames());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = ep[i];

  simple_setup->setStartAndGoalStates(start_state, goal_state);

  // Setup state checking functionality
  if (svc != nullptr)
    simple_setup->setStateValidityChecker(svc);

  // Setup motion validation (i.e. collision checking)
  if (mv != nullptr)
  {
    simple_setup->getSpaceInformation()->setMotionValidator(mv);
  }
  else
  {
    if (collision_check)
    {
      ompl::base::MotionValidatorPtr mv;
      if (collision_continuous)
      {
        mv = std::make_shared<ContinuousMotionValidator>(
            simple_setup->getSpaceInformation(), env, kin, collision_safety_margin);
        simple_setup->getSpaceInformation()->setMotionValidator(mv);
      }
      else
      {
        mv = std::make_shared<DiscreteMotionValidator>(
            simple_setup->getSpaceInformation(), env, kin, collision_safety_margin);
        simple_setup->getSpaceInformation()->setMotionValidator(mv);
      }
    }
  }

  // make sure the planners run until the time limit, and get the best possible solution
  if (optimization_objective_allocator)
  {
    simple_setup->getProblemDefinition()->setOptimizationObjective(
        optimization_objective_allocator(simple_setup->getSpaceInformation()));
  }

  return true;
}

tesseract_common::TrajArray OMPLPlannerFreespaceConfig::getTrajectory() const
{
  const auto& path = this->simple_setup->getSolutionPath();
  const auto n_points = static_cast<long>(path.getStateCount());
  const auto dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract_common::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
  {
#ifndef OMPL_LESS_1_4_0
    if (constraint)
    {
      const Eigen::Map<Eigen::VectorXd>& x =
          *path.getState(static_cast<unsigned>(i))->template as<ompl::base::ConstrainedStateSpace::StateType>();
      result.row(i) = x;
    }
    else
    {
      const auto& s =
          path.getState(static_cast<unsigned>(i))->template as<ompl::base::RealVectorStateSpace::StateType>();
      for (long j = 0; j < dof; ++j)
        result(i, j) = s->values[j];
    }
#else
    const auto& s = path.getState(static_cast<unsigned>(i))->as<ompl::base::RealVectorStateSpace::StateType>();
    for (long j = 0; j < dof; ++j)
      result(i, j) = s->values[j];
#endif
  }

  return result;
}

ompl::base::StateSamplerPtr
OMPLPlannerFreespaceConfig::allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const
{
  return std::make_shared<WeightedRealVectorStateSampler>(space, weights);
}

}  // namespace tesseract_motion_planners
