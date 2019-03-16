/**
 * @file trajopt_freespace_planner.h
 * @brief Tesseract ROS TrajOpt Freespace planner
 *
 * @author Matthew Powelson
 * @date February 26, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_PLANNING_TRAJOPT_FREESPACE_PLANNER_H
#define TESSERACT_PLANNING_TRAJOPT_FREESPACE_PLANNER_H

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_planning/basic_planner.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/waypoint_definitions.h>

namespace tesseract
{
namespace tesseract_planning
{
/**
 * @brief Config to setup freespace planner. Specify the start and end position. Freespace motion can be defined between
 * either joint space positions or cartesian positions
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: kin_, env_, end_effector
 *
 */
struct TrajOptFreespacePlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajOptFreespacePlannerConfig() {}
  virtual ~TrajOptFreespacePlannerConfig() {}
  /** @brief Determines the constraint placed at the start of the trajectory */
  WaypointPtr start_waypoint_;
  /** @brief Determines the constraint placed at the end of the trajectory */
  WaypointPtr end_waypoint_;
  // TODO: These are waypoints the planner must hit in between
  std::vector<Waypoint> intermediate_waypoints;
  /** @brief The total number of timesteps used in the freespace motion. Default: 20 */
  int num_steps_ = 20;

  /** @brief Selects the type of initialization used for freespace motion. If GIVEN_TRAJ, then the seed_trajectory_ must be set */
  trajopt::InitInfo::Type init_type_ = trajopt::InitInfo::STATIONARY;
  /** @brief The trajectory used as the optimization seed when init_type_ is set to GIVEN_TRAJ */
  trajopt::TrajArray seed_trajectory_;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check_ = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous_ = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin_ = 0.025;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities_ = true;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations_ = true;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks_ = true;
  /** @brief If true, add a callback to plot each iteration */
  bool plot_callback_ = false;

  /** @brief Basic Kinematics object. ***REQUIRED*** */
  tesseract::BasicKinConstPtr kin_;
  /** @brief Basic Environment object. ***REQUIRED*** */
  tesseract::BasicEnvConstPtr env_;
  /** @brief This is the link used for the cartesian positions ***REQUIRED*** */
  std::string end_effector_;
  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator_;

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params_;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks_;
};

/**
 * @brief This planner is intended to provide an easy to use interface to TrajOpt for freespace planning. It is made to
 * take a start and end point and automate the generation of the TrajOpt problem.
 */
class TrajOptFreespacePlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptFreespacePlanner()
  {
    name_ = "TRAJOPT_FREESPACE";

    // Error Status Codes
    status_code_map_[-1] = "Invalid config data format";
    status_code_map_[-2] = "Failed to parse config data";
    status_code_map_[-3] = "";

    // Converge Status Codes
  }

  // TODO: Not sure what to do here, but this is defined in the base as virtual
  bool solve(PlannerResponse& response) override { return false; };
  /**
   * @brief Sets up the TrajOpt problem then solves using TrajOptPlanner. It is intended to simplify setting up and
   * solving freespace motion problems.
   *
   * This planner (and the associated config) does not expose all of the available configuration data in TrajOpt. This
   * is done to simplify the interface. However, many problems may require more specific setups. In that case, the
   * source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param config Configures the freespace planner
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response, TrajOptFreespacePlannerConfig& config);

  bool terminate() override;

  void clear() override;
};
}  // namespace tesseract_planning
}  // namespace tesseract
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
