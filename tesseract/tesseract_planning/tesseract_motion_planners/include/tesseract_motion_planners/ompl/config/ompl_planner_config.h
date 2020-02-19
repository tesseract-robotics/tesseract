/**
 * @file ompl_planner_config.h
 * @brief Tesseract OMPL planner config.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/tesseract.h>

namespace tesseract_motion_planners
{
/** @brief The OMPLPlannerConfig struct */
struct OMPLPlannerConfig
{
  using Ptr = std::shared_ptr<OMPLPlannerConfig>;
  using ConstPtr = std::shared_ptr<const OMPLPlannerConfig>;

  explicit OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
                             std::string manipulator,
                             std::vector<OMPLPlannerConfigurator::ConstPtr> planners);

  explicit OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
                             std::string manipulator,
                             std::vector<OMPLPlannerConfigurator::ConstPtr> planners,
                             ompl::geometric::SimpleSetupPtr simple_setup);

  virtual ~OMPLPlannerConfig() = default;
  OMPLPlannerConfig(const OMPLPlannerConfig&) = default;
  OMPLPlannerConfig& operator=(const OMPLPlannerConfig&) = default;
  OMPLPlannerConfig(OMPLPlannerConfig&&) noexcept = default;
  OMPLPlannerConfig& operator=(OMPLPlannerConfig&&) noexcept = default;

  /**
   * @brief Generates the OMPL problem and saves the result internally
   * @return True on success, false on failure
   */
  virtual bool generate();

  /**
   * @brief Convert the path stored in simple_setup to tesseract trajectory
   * This is required because the motion planner is not aware of the state space type.
   * @return Tesseract Trajectory
   */
  tesseract_common::TrajArray getTrajectory() const;

  /** @brief Max planning time allowed in seconds */
  double planning_time = 5.0;
  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;
  /**
   * @brief Simplify trajectory.
   *
   * Note: If set to true it ignores n_output_states and returns the simplest trajectory.
   */
  bool simplify = true;
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
  bool optimize = false;

  /** @brief OMPL problem to be solved ***REQUIRED*** */
  ompl::geometric::SimpleSetupPtr simple_setup;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator used for path planning ***REQUIRED*** */
  std::string manipulator;

  /**
   * @brief The planner configurators ***REQUIRED***
   *
   * This will create a new thead for each planner configurator provided. T
   */
  std::vector<OMPLPlannerConfigurator::ConstPtr> planners;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid in post checking of trajectory returned by trajopt.
   *
   * The resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent()
   *
   * Note: The planner takes the conservative of either longest_valid_segment_fraction or longest_valid_segment_length.
   */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. If norm(state1 - state0) > longest_valid_segment_length.
   *
   * Note: This gets converted to longest_valid_segment_fraction.
   *       longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
   */
  double longest_valid_segment_length = 0.5;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_H
