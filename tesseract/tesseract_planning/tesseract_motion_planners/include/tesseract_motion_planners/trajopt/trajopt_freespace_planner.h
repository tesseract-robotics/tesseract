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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_FREESPACE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_FREESPACE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{

class TrajOptFreespacePlannerStatusCategory;

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
  /** @brief Determines the constraint placed at the start of the trajectory */
  Waypoint::Ptr start_waypoint_;
  /** @brief Determines the constraint placed at the end of the trajectory */
  Waypoint::Ptr end_waypoint_;
  // TODO: These are waypoints the planner must hit in between
  std::vector<Waypoint::Ptr> intermediate_waypoints;
  /** @brief The total number of timesteps used in the freespace motion. Default: 20 */
  int num_steps_ = 20;

  /** @brief Selects the type of initialization used for freespace motion. If GIVEN_TRAJ, then the seed_trajectory_ must
   * be set */
  trajopt::InitInfo::Type init_type_ = trajopt::InitInfo::STATIONARY;
  /** @brief The trajectory used as the optimization seed when init_type_ is set to GIVEN_TRAJ */
  trajopt::TrajArray seed_trajectory_;
  /** @brief This joint waypoint represents the desired configuration (Optional). This is not guaranteed, but a small
   * equality cost is set to the joint position for all points to pull the optimization in that direction.
   *
   * An example use case is setting it to be at the center of the joint limits. This tends to pull the robot away from
   * singularities */
  JointWaypoint::ConstPtr configuration_ = nullptr;

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

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract_;
  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator_;
  /** @brief This is the link used for the cartesian positions ***REQUIRED*** */
  std::string link_;
  /** @brief A transform applied to link_ used for the cartesian position */
  Eigen::Isometry3d tcp_ = Eigen::Isometry3d::Identity();

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params_;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks_;
};

/**
 * @brief This planner is intended to provide an easy to use interface to TrajOpt for freespace planning. It is made to
 * take a start and end point and automate the generation of the TrajOpt problem.
 */
class TrajOptFreespacePlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptFreespacePlanner(std::string name = "TRAJOPT_FREESPACE");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const TrajOptFreespacePlannerConfig& config);

  /**
   * @brief Sets up the TrajOpt problem then solves using TrajOptMotionPlanner. It is intended to simplify setting up
   * and solving freespace motion problems.
   *
   * This planner (and the associated config passed to the setConfiguration) does not expose all of the available
   * configuration data in TrajOpt. This is done to simplify the interface. However, many problems may require more
   * specific setups. In that case, the source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(PlannerResponse& response) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

protected:
  tesseract_motion_planners::TrajOptMotionPlanner planner_; /** @brief The trajopt planner */
  std::shared_ptr<trajopt::ProblemConstructionInfo> pci_;
  std::shared_ptr<TrajOptFreespacePlannerConfig> config_;
  std::shared_ptr<const TrajOptFreespacePlannerStatusCategory> status_category_; /** @brief The plannsers status codes */
};

class TrajOptFreespacePlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptFreespacePlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 0,
    IsNotConfigured = -1
  };

private:
  std::string name_;
};


}  // namespace tesseract_motion_planners
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
