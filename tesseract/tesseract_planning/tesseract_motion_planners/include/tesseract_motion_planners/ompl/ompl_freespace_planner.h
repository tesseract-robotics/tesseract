/**
 * @file ompl_freespace_planner.h
 * @brief Tesseract OMPL freespace planner.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_motion_planners/ompl/ompl_settings.h>
#include <tesseract_motion_planners/ompl/ompl_freespace_planner_status_category.h>

namespace tesseract_motion_planners
{
template <typename PlannerType>
struct OMPLFreespacePlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Determines the constraint placed at the start of the trajectory
   *
   * The planner currently only support joint waypoints. Need to expose the ability
   * to create a state sampler.
   */
  Waypoint::Ptr start_waypoint;

  /**
   * @brief Determines the constraint placed at the end of the trajectory
   *
   * The planner currently only support joint waypoints. Need to expose the ability
   * to create a state sampler.
   */
  Waypoint::Ptr end_waypoint;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;
  /** @brief Max planning time allowed */
  double planning_time = 5.0;
  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;
  /** @brief The number of threads to use */
  int num_threads = 1;
  /** @brief Simplify trajectory */
  bool simplify = true;
  /**
   * @brief Number of states in the output trajectory
   *   Note: This is ignored if the trajectory is simplified
   *   Note: The trajectory can be longer if original trajectory is longer and reducing the number of states causes
   *         the solution to be invalid.
   */
  int n_output_states = 20;
  /** @brief This scales the variables search space. Must be same size as number of joints.
   *         If empty it defaults to all ones */
  Eigen::VectorXd weights;

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. The resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent()
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

  /** @brief Planner settings */
  OMPLSettings<PlannerType> settings;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;

  /** @brief The ompl state validity checker. If nullptr it uses OMPLFreespacePlanner::isStateValid. */
  ompl::base::StateValidityCheckerFn svc;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  ompl::base::MotionValidatorPtr mv;
};

/**
 * @brief This planner is intended to provide an easy to use interface to OMPL for freespace planning. It is made to
 * take a start and end point and automate the generation of the OMPL problem.
 */
template <typename PlannerType>
class OMPLFreespacePlanner : public MotionPlanner
{
public:
  /** @brief Construct a planner */
  OMPLFreespacePlanner(std::string name = "OMPL_FREESPACE");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const OMPLFreespacePlannerConfig<PlannerType>& config);

  /**
   * @brief Sets up the OMPL problem then solves. It is intended to simplify setting up
   * and solving freespace motion problems.
   *
   * This planner (and the associated config passed to the setConfiguration) does not expose all of the available
   * configuration data in OMPL. This is done to simplify the interface. However, many problems may require more
   * specific setups. In that case, the source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of OMPL.
   * @return true if valid solution was found
   */
  tesseract_common::StatusCode solve(PlannerResponse& response, bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

private:
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const;

protected:
  /** @brief The ompl planner planner */
  std::shared_ptr<OMPLFreespacePlannerConfig<PlannerType>> config_;

  /** @brief The planners status codes */
  std::shared_ptr<const OMPLFreespacePlannerStatusCategory> status_category_;

  /** @brief The ompl planner motion validator */
  ompl::base::MotionValidatorPtr motion_validator_;

  /** @brief The ompl planner simple setup */
  ompl::geometric::SimpleSetupPtr simple_setup_;

  /** @brief The tesseract kinematics object */
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;

  /** @brief The mapping of environment links to kinematics links */
  tesseract_environment::AdjacencyMap::ConstPtr adj_map_;

  /** @brief The discrete contact manager */
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;

  /** @brief The continuous contact manager */
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
