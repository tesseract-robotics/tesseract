/**
 * @file trajopt_taskflow.h
 * @brief TrajOpt Graph Taskflow
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
#ifndef TESSERACT_PROCESS_MANAGERS_TRAJOPT_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_TRAJOPT_TASKFLOW_H

#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_process_managers/visibility_control.h>

namespace tesseract_planning
{
TESSERACT_PROCESS_MANAGERS_PUBLIC GraphTaskflow::UPtr createTrajOptTaskflow(
    bool create_seed,
    const SimplePlannerPlanProfileMap& simple_plan_profiles = SimplePlannerPlanProfileMap(),
    const SimplePlannerCompositeProfileMap& simple_composite_profiles = SimplePlannerCompositeProfileMap(),
    const TrajOptPlanProfileMap& trajopt_plan_profiles = TrajOptPlanProfileMap(),
    const TrajOptCompositeProfileMap& trajopt_composite_profiles = TrajOptCompositeProfileMap());
}
#endif  // TESSERACT_PROCESS_MANAGERS_TRAJOPT_TASKFLOW_H
