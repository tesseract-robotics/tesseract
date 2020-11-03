/**
 * @file simple_freespace_taskflow.h
 * @brief Simple Freespace Graph Taskflow
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
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/discrete_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>
#include <tesseract_process_managers/process_generators/seed_min_length_process_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>

namespace tesseract_planning
{
GraphTaskflow::UPtr createFreespaceTaskflowDefault(const FreespaceTaskflowParams& params)
{
  auto graph = std::make_unique<GraphTaskflow>();

  ///////////////////
  /// Add Process ///
  ///////////////////

  // Setup Interpolator
  int interpolator_idx{ -1 };
  if (params.enable_simple_planner)
  {
    auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
    interpolator->plan_profiles = params.simple_plan_profiles;
    interpolator->composite_profiles = params.simple_composite_profiles;
    auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
    interpolator_idx = graph->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  auto seed_min_length_generator = std::make_unique<SeedMinLengthProcessGenerator>();
  int seed_min_length_idx = graph->addNode(std::move(seed_min_length_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  ompl_planner->plan_profiles = params.ompl_plan_profiles;
  auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
  int ompl_idx = graph->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::TASK);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
  trajopt_planner->plan_profiles = params.trajopt_plan_profiles;
  trajopt_planner->composite_profiles = params.trajopt_composite_profiles;
  auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
  int trajopt_idx = graph->addNode(std::move(trajopt_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Add Final Continuous Contact Check of trajectory
  int contact_check_idx{ -1 };
  if (params.enable_post_contact_continuous_check)
  {
    auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
    contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }
  else if (params.enable_post_contact_discrete_check)
  {
    auto contact_check_generator = std::make_unique<DiscreteContactCheckProcessGenerator>();
    contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }

  // Time parameterization trajectory
  int time_parameterization_idx{ -1 };
  if (params.enable_time_parameterization)
  {
    auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
    time_parameterization_idx =
        graph->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }
  /////////////////
  /// Add Edges ///
  /////////////////
  auto ON_NONE = GraphTaskflow::SourceChannel::NONE;
  auto ON_SUCCESS = GraphTaskflow::SourceChannel::ON_SUCCESS;
  auto ON_FAILURE = GraphTaskflow::SourceChannel::ON_FAILURE;
  auto PROCESS_NODE = GraphTaskflow::DestinationChannel::PROCESS_NODE;
  auto ERROR_CALLBACK = GraphTaskflow::DestinationChannel::ERROR_CALLBACK;
  auto DONE_CALLBACK = GraphTaskflow::DestinationChannel::DONE_CALLBACK;

  if (params.enable_simple_planner)
  {
    graph->addEdge(interpolator_idx, ON_SUCCESS, seed_min_length_idx, PROCESS_NODE);
    graph->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  graph->addEdge(seed_min_length_idx, ON_SUCCESS, ompl_idx, PROCESS_NODE);
  graph->addEdge(seed_min_length_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(ompl_idx, ON_NONE, trajopt_idx, PROCESS_NODE);

  graph->addEdge(trajopt_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    graph->addEdge(trajopt_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  else if (params.enable_time_parameterization)
    graph->addEdge(trajopt_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  else
    graph->addEdge(trajopt_idx, ON_SUCCESS, -1, DONE_CALLBACK);

  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
  {
    graph->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    if (params.enable_time_parameterization)
      graph->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
    else
      graph->addEdge(contact_check_idx, ON_SUCCESS, -1, DONE_CALLBACK);
  }

  if (params.enable_time_parameterization)
  {
    graph->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
    graph->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  return graph;
}

GraphTaskflow::UPtr createFreespaceTaskflowTrajOptFirst(const FreespaceTaskflowParams& params)
{
  auto graph = std::make_unique<GraphTaskflow>();

  ///////////////////
  /// Add Process ///
  ///////////////////

  // Setup Interpolator
  int interpolator_idx{ -1 };
  if (params.enable_simple_planner)
  {
    auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
    interpolator->plan_profiles = params.simple_plan_profiles;
    interpolator->composite_profiles = params.simple_composite_profiles;
    auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
    interpolator_idx = graph->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  auto seed_min_length_generator = std::make_unique<SeedMinLengthProcessGenerator>();
  int seed_min_length_idx = graph->addNode(std::move(seed_min_length_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
  trajopt_planner->plan_profiles = params.trajopt_plan_profiles;
  trajopt_planner->composite_profiles = params.trajopt_composite_profiles;
  auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
  int trajopt_idx = graph->addNode(std::move(trajopt_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  ompl_planner->plan_profiles = params.ompl_plan_profiles;
  auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
  int ompl_idx = graph->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup TrajOpt 2
  auto trajopt_planner2 = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner2->problem_generator = &DefaultTrajoptProblemGenerator;
  trajopt_planner2->plan_profiles = params.trajopt_plan_profiles;
  trajopt_planner2->composite_profiles = params.trajopt_composite_profiles;
  auto trajopt_generator2 = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner2);
  int trajopt_idx2 = graph->addNode(std::move(trajopt_generator2), GraphTaskflow::NodeType::CONDITIONAL);

  // Add Final Continuous Contact Check of trajectory
  int contact_check_idx{ -1 };
  if (params.enable_post_contact_continuous_check)
  {
    auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
    contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }
  else if (params.enable_post_contact_discrete_check)
  {
    auto contact_check_generator = std::make_unique<DiscreteContactCheckProcessGenerator>();
    contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }

  // Time parameterization trajectory
  int time_parameterization_idx{ -1 };
  if (params.enable_time_parameterization)
  {
    auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
    time_parameterization_idx =
        graph->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);
  }
  /////////////////
  /// Add Edges ///
  /////////////////
  auto ON_SUCCESS = GraphTaskflow::SourceChannel::ON_SUCCESS;
  auto ON_FAILURE = GraphTaskflow::SourceChannel::ON_FAILURE;
  auto PROCESS_NODE = GraphTaskflow::DestinationChannel::PROCESS_NODE;
  auto ERROR_CALLBACK = GraphTaskflow::DestinationChannel::ERROR_CALLBACK;
  auto DONE_CALLBACK = GraphTaskflow::DestinationChannel::DONE_CALLBACK;

  if (params.enable_simple_planner)
  {
    graph->addEdge(interpolator_idx, ON_SUCCESS, seed_min_length_idx, PROCESS_NODE);
    graph->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  graph->addEdge(seed_min_length_idx, ON_SUCCESS, trajopt_idx, PROCESS_NODE);
  graph->addEdge(seed_min_length_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(trajopt_idx, ON_FAILURE, ompl_idx, PROCESS_NODE);
  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    graph->addEdge(trajopt_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  else if (params.enable_time_parameterization)
    graph->addEdge(trajopt_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  else
    graph->addEdge(trajopt_idx, ON_SUCCESS, -1, DONE_CALLBACK);

  graph->addEdge(ompl_idx, ON_SUCCESS, trajopt_idx2, PROCESS_NODE);
  graph->addEdge(ompl_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(trajopt_idx2, ON_FAILURE, -1, ERROR_CALLBACK);
  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    graph->addEdge(trajopt_idx2, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  else if (params.enable_time_parameterization)
    graph->addEdge(trajopt_idx2, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  else
    graph->addEdge(trajopt_idx2, ON_SUCCESS, -1, DONE_CALLBACK);

  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
  {
    graph->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    if (params.enable_time_parameterization)
      graph->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
    else
      graph->addEdge(contact_check_idx, ON_SUCCESS, -1, DONE_CALLBACK);
  }

  if (params.enable_time_parameterization)
  {
    graph->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
    graph->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  return graph;
}

GraphTaskflow::UPtr createFreespaceTaskflow(FreespaceTaskflowParams params)
{
  switch (static_cast<int>(params.type))
  {
    case static_cast<int>(FreespaceTaskflowType::DEFAULT):
    {
      return createFreespaceTaskflowDefault(params);
    }
    case static_cast<int>(FreespaceTaskflowType::TRAJOPT_FIRST):
    {
      return createFreespaceTaskflowTrajOptFirst(params);
    }
    default:
    {
      return createFreespaceTaskflowDefault(params);
    }
  }
}

}  // namespace tesseract_planning
