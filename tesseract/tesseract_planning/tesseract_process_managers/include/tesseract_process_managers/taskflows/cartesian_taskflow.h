/**
 * @file simple_cartesian_taskflow.h
 * @brief Simple Cartesian Graph Taskflow
 *
 * @author Levi Armstrong
 * @date August 27, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H

#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

namespace tesseract_planning
{
struct CartesianTaskflowParams
{
  bool enable_simple_planner{ true };
  bool enable_post_contact_discrete_check{ false };
  bool enable_post_contact_continuous_check{ true };
  bool enable_time_parameterization{ true };
  SimplePlannerPlanProfileMap simple_plan_profiles;
  SimplePlannerCompositeProfileMap simple_composite_profiles;
  DescartesPlanProfileMap<double> descartes_plan_profiles;
  TrajOptPlanProfileMap trajopt_plan_profiles;
  TrajOptCompositeProfileMap trajopt_composite_profiles;
};

GraphTaskflow::UPtr createCartesianTaskflow(CartesianTaskflowParams params);
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_CARTESIAN_TASKFLOW_H
