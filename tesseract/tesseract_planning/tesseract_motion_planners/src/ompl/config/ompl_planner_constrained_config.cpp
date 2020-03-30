/**
 * @file ompl_planner_constrained_config.cpp
 * @brief Tesseract OMPL planner constrained config implementation.
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

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_constrained_config.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>
#include <tesseract_motion_planners/ompl/compound_state_validator.h>

namespace tesseract_motion_planners
{
/** @brief Construct a basic planner */
OMPLPlannerConstrainedConfig::OMPLPlannerConstrainedConfig(tesseract::Tesseract::ConstPtr tesseract,
                                                           std::string manipulator)
  : OMPLPlannerFreespaceConfig(std::move(tesseract), std::move(manipulator))
{
}

OMPLPlannerConstrainedConfig::OMPLPlannerConstrainedConfig(tesseract::Tesseract::ConstPtr tesseract,
                                                           std::string manipulator,
                                                           std::vector<OMPLPlannerConfigurator::ConstPtr> planners)
  : OMPLPlannerFreespaceConfig(std::move(tesseract), std::move(manipulator), std::move(planners))
{
}

bool OMPLPlannerConstrainedConfig::generate()
{
  // Check that parameters are valid
  if (tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerConstrainedConfig: tesseract is a required parameter and has not been set");
    return false;
  }

  tesseract_kinematics::ForwardKinematics::Ptr kin =
      tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerConstrainedConfig: failed to get kinematics object for manipulator: %s.",
                            manipulator.c_str());
    return false;
  }

  if (weights.size() == 0)
  {
    weights = Eigen::VectorXd::Ones(kin->numJoints());
  }
  else if (weights.size() != kin->numJoints())
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerConstrainedConfig: The weights must be the same length as the number of "
                            "joints or "
                            "have a length of zero!");
    return false;
  }

  if (constraint == nullptr)
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerConstrainedConfig: No constraint was provided.");
    return false;
  }

  const tesseract_environment::Environment::ConstPtr& env = tesseract->getEnvironmentConst();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

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

  state_space_ptr = std::make_shared<ompl::base::ProjectedStateSpace>(rss, constraint);
  extractor = tesseract_motion_planners::ConstrainedStateSpaceExtractor;

  // Setup Longest Valid Segment
  processLongestValidSegment(state_space_ptr);

  // Create Simple Setup from state space
  auto csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(state_space_ptr);
  this->simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(csi);

  // Setup start and goal state
  if (!processStartAndGoalState(env, kin))
    return false;

  // Setup state checking functionality
  ompl::base::StateValidityCheckerPtr svc_without_collision = processStateValidator(env, kin);

  // Setup motion validation (i.e. collision checking)
  processMotionValidator(svc_without_collision, env, kin);

  // make sure the planners run until the time limit, and get the best possible solution
  processOptimizationObjective();

  simple_setup->setup();
  return true;
}

}  // namespace tesseract_motion_planners
