/**
 * @file ompl_motion_planner.h
 * @brief Tesseract OMPL motion planner.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_OMPL_MOTION_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <utility>
#include <type_traits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_config.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>

namespace tesseract_planning
{
/**
 * @brief This planner is intended to provide an easy to use interface to OMPL for freespace planning. It is made to
 * take a start and end point and automate the generation of the OMPL problem.
 */
class OMPLMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a planner */
  OMPLMotionPlanner(std::string name = "OMPL");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(OMPLMotionPlannerConfig::Ptr config);

  /**
   * @brief Sets up the OMPL problem then solves. It is intended to simplify setting up
   * and solving freespace motion problems.
   *
   * This planner leverages OMPL ParallelPlan which supports being called multiple times. So you are able to
   * setConfiguration and keep calling solve and with each solve it will continue to build the existing planner graph.
   * If you want to start with a clean graph you must call setConfiguration() before calling solve again.
   *
   * This planner (and the associated config passed to the setConfiguration) does not expose all of the available
   * configuration data in OMPL. This is done to simplify the interface. However, many problems may require more
   * specific setups. In that case, the source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of OMPL.
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Flag for printing more detailed planning information
   * @return true if valid solution was found
   */
  tesseract_common::StatusCode solve(PlannerResponse& response,
                                     PostPlanCheckType check_type = PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                     bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

protected:
  /** @brief The ompl planner planner */
  typename OMPLMotionPlannerConfig::Ptr config_;

  /** @brief The planners status codes */
  std::shared_ptr<const OMPLMotionPlannerStatusCategory> status_category_;

  /** @brief OMPL Parallel planner */
  std::shared_ptr<ompl::tools::ParallelPlan> parallel_plan_;

  /** @brief The continuous contact manager */
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_MOTION_PLANNER_H
