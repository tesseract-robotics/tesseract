/**
 * @file descartes_motion_planner.h
 * @brief Tesseract ROS Descartes planner
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
#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
class DescartesMotionPlannerStatusCategory;

template <typename FloatType>
struct DescartesMotionPlannerConfig
{
  DescartesMotionPlannerConfig(tesseract::Tesseract::ConstPtr tesseract_ptr,
                               const std::vector<std::string> active_link_names,
                               const std::vector<std::string> joint_names,
                               const typename descartes_light::EdgeEvaluator<FloatType>::Ptr& edge_evaluator,
                               std::vector<descartes_core::TimingConstraint<FloatType>> timing_constraint,
                               std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> samplers,
                               const std::vector<Waypoint::Ptr>& waypoints)
    : tesseract(tesseract_ptr)
    , active_link_names(std::move(active_link_names))
    , joint_names(std::move(joint_names))
    , edge_evaluator(edge_evaluator)
    , timing_constraint(timing_constraint)
    , samplers(samplers)
    , waypoints(waypoints)
  {
  }

  virtual ~DescartesMotionPlannerConfig() = default;

  const tesseract::Tesseract::ConstPtr tesseract;
  const std::vector<std::string> active_link_names;
  const std::vector<std::string> joint_names;
  const typename descartes_light::EdgeEvaluator<FloatType>::Ptr edge_evaluator;
  const std::vector<descartes_core::TimingConstraint<FloatType>> timing_constraint;
  const std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> samplers;
  const std::vector<Waypoint::Ptr> waypoints;
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d world_to_base = Eigen::Isometry3d::Identity();
  int num_threads = descartes_light::Solver<double>::getMaxThreads();
};

using DescartesMotionPlannerConfigD = DescartesMotionPlannerConfig<double>;
using DescartesMotionPlannerConfigF = DescartesMotionPlannerConfig<float>;

template <typename FloatType>
class DescartesMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesMotionPlanner(std::string name = "DESCARTES");

  ~DescartesMotionPlanner() override {}

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const DescartesMotionPlannerConfig<FloatType>& config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(PlannerResponse& response, const bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

private:
  std::shared_ptr<DescartesMotionPlannerConfig<FloatType>> config_; /**< @brief The planners configuration */
  std::shared_ptr<const DescartesMotionPlannerStatusCategory> status_category_; /**< @brief The planners status codes */
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

/**
 * @brief The Descartes motion planner status category
 *
 * It contains both successfull and error status codes.

 */
class DescartesMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  DescartesMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    ErrorIsNotConfigured = -1,
    ErrorFailedToParseConfig = -2,
    ErrorFailedToBuildGraph = -3,
    ErrorFailedToFindValidSolution = -4,
    ErrorFoundValidSolutionInCollision = -5
  };

private:
  std::string name_; /**< @brief The name of the status category */
};

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
