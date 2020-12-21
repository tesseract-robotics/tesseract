/**
 * @file ompl_problem.h
 * @brief Tesseract OMPL problem definition
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROBLEM_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/types.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_environment/core/environment.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::OMPLProblem)
%ignore tesseract_planning::OMPLProblem::extractor;
#endif  // SWIG

namespace tesseract_planning
{
struct OMPLProblem;

using StateSamplerAllocator =
    std::function<ompl::base::StateSamplerPtr(const ompl::base::StateSpace*, const OMPLProblem&)>;

using OptimizationObjectiveAllocator =
    std::function<ompl::base::OptimizationObjectivePtr(const ompl::base::SpaceInformationPtr&, const OMPLProblem&)>;

using StateValidityCheckerAllocator =
    std::function<ompl::base::StateValidityCheckerPtr(const ompl::base::SpaceInformationPtr&, const OMPLProblem&)>;

using MotionValidatorAllocator =
    std::function<ompl::base::MotionValidatorPtr(const ompl::base::SpaceInformationPtr&, const OMPLProblem&)>;

enum class OMPLProblemStateSpace
{
  REAL_STATE_SPACE,
#ifndef OMPL_LESS_1_4_0
  REAL_CONSTRAINTED_STATE_SPACE,
#endif
  SE3_STATE_SPACE,
};

struct OMPLProblem
{
  using Ptr = std::shared_ptr<OMPLProblem>;
  using ConstPtr = std::shared_ptr<const OMPLProblem>;
  using UPtr = std::unique_ptr<OMPLProblem>;
  using ConstUPtr = std::unique_ptr<const OMPLProblem>;

  // These are required for Tesseract to configure ompl
  tesseract_environment::Environment::ConstPtr env;
  tesseract_environment::EnvState::ConstPtr env_state;
  tesseract_environment::StateSolver::Ptr state_solver;

  // This is used to verify that start and goal states are not in collision
  tesseract_collision::DiscreteContactManager::Ptr contact_checker;

  // Problem Configuration
  OMPLProblemStateSpace state_space{ OMPLProblemStateSpace::REAL_STATE_SPACE };

  // Kinematic Objects
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_fwd_kin;
  tesseract_kinematics::InverseKinematics::ConstPtr manip_inv_kin;

  /** @brief Max planning time allowed in seconds */
  double planning_time = 5.0;

  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;

  /**
   * @brief Simplify trajectory.
   *
   * Note: If set to true it ignores n_output_states and returns the simplest trajectory.
   */
  bool simplify = false;

  /**
   * @brief Number of states in the output trajectory
   *   Note: This is ignored if the simplify is set to true.
   *   Note: The trajectory can be longer if original trajectory is longer and reducing the number of states causes
   *         the solution to be invalid.
   */
  int n_output_states = 20;

  /**
   * @brief This uses all available planning time to create the most optimized trajectory given the objective function.
   *
   * This is required because not all OMPL planners are optimize graph planners. If the planner you choose is an
   * optimize graph planner then setting this to true has no affect. In the case of non-optimize planners they still
   * use the OptimizeObjective function but only when searching the graph to find the most optimize solution based
   * on the provided optimize objective function. In the case of these type of planners like RRT and RRTConnect if set
   * to true it will leverage all planning time to keep finding solutions up to your max solutions count to find the
   * most optimal solution.
   */
  bool optimize = true;

  /** @brief OMPL problem to be solved ***REQUIRED*** */
  ompl::geometric::SimpleSetupPtr simple_setup;

  /**
   * @brief The planner configurators ***REQUIRED***
   *
   * This will create a new thead for each planner configurator provided. T
   */
  std::vector<OMPLPlannerConfigurator::ConstPtr> planners;

  /**
   * @bried This will extract an Eigen::VectorXd from the OMPL State ***REQUIRED***
   */
  OMPLStateExtractor extractor;

  /**
   * @brief Convert the path stored in simple_setup to tesseract trajectory
   * This is required because the motion planner is not aware of the state space type.
   * @return Tesseract Trajectory
   */
  tesseract_common::TrajArray getTrajectory() const;
};

}  // namespace tesseract_planning

#ifdef SWIG
%template(OMPLProblems) std::vector<std::shared_ptr<tesseract_planning::OMPLProblem>>;
#endif

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROBLEM_H
