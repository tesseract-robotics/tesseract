/**
 * @file ompl_taskflow.cpp
 * @brief OMPL Graph Taskflow
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
#include <tesseract_process_managers/taskflows/ompl_taskflow.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

namespace tesseract_planning
{
GraphTaskflow::UPtr createOMPLTaskflow(bool create_seed,
                                       const SimplePlannerPlanProfileMap& simple_plan_profiles,
                                       const SimplePlannerCompositeProfileMap& simple_composite_profiles,
                                       const OMPLPlanProfileMap& ompl_plan_profiles)
{
  auto graph = std::make_unique<GraphTaskflow>();

  ///////////////////
  /// Add Process ///
  ///////////////////

  // Setup Interpolator
  int interpolator_idx{ -1 };
  if (create_seed)
  {
    auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
    interpolator->plan_profiles = simple_plan_profiles;
    interpolator->composite_profiles = simple_composite_profiles;
    auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
    interpolator_idx = graph->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  ompl_planner->plan_profiles = ompl_plan_profiles;
  auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
  int ompl_idx = graph->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Add Final Continuous Contact Check of trajectory
  auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
  int contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Time parameterization trajectory
  auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
  int time_parameterization_idx =
      graph->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);

  /////////////////
  /// Add Edges ///
  /////////////////
  auto ON_SUCCESS = GraphTaskflow::SourceChannel::ON_SUCCESS;
  auto ON_FAILURE = GraphTaskflow::SourceChannel::ON_FAILURE;
  auto PROCESS_NODE = GraphTaskflow::DestinationChannel::PROCESS_NODE;
  auto ERROR_CALLBACK = GraphTaskflow::DestinationChannel::ERROR_CALLBACK;
  auto DONE_CALLBACK = GraphTaskflow::DestinationChannel::DONE_CALLBACK;

  if (create_seed)
  {
    graph->addEdge(interpolator_idx, ON_SUCCESS, ompl_idx, PROCESS_NODE);
    graph->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  graph->addEdge(ompl_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  graph->addEdge(ompl_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  graph->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
  graph->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  return graph;
}
}  // namespace tesseract_planning
