/**
 * @file ompl_trajopt_freespace_planner.h
 * @brief Tesseract OMPL TrajOpt Freespace Planner
 *
 * @author Michael Ripperger
 * @date October 3, 2019
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
#ifndef TESSERACT_MOTION_PLANNERS_HYBRID_OMPL_TRAJOPT_FREESPACE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_HYBRID_OMPL_TRAJOPT_FREESPACE_PLANNER_H

#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>

namespace tesseract_motion_planners
{
template <typename PlannerType>
class OMPLTrajOptFreespacePlanner : public MotionPlanner
{
public:
  OMPLTrajOptFreespacePlanner(std::string name = "OMPL_TRAJOPT_FREESPACE_PLANNER");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(OMPLFreespacePlannerConfig<PlannerType> ompl_config,
                        std::shared_ptr<TrajOptPlannerFreespaceConfig> trajopt_config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
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
  std::shared_ptr<TrajOptPlannerFreespaceConfig> trajopt_config_;
  TrajOptMotionPlanner trajopt_planner_;

  OMPLFreespacePlanner<PlannerType> ompl_planner_;

  std::shared_ptr<const tesseract_common::GeneralStatusCategory> status_category_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_HYBRID_OMPL_TRAJOPT_FREESPACE_PLANNER_H
