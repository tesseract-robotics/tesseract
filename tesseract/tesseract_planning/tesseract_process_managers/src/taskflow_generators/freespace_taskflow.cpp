/**
 * @file freespace_taskflow.cpp
 * @brief Freespace Graph Taskflow
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/discrete_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>
#include <tesseract_process_managers/process_generators/seed_min_length_process_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

using namespace tesseract_planning;

FreespaceTaskflow::FreespaceTaskflow(FreespaceTaskflowParams params, std::string name)
  : name_(name), params_(params), generator_(std::make_unique<GraphTaskflow>(name_))
{
  if (params_.type == FreespaceTaskflowType::TRAJOPT_FIRST)
  {
    // Setup Interpolator
    int interpolator_idx{ -1 };
    if (params.enable_simple_planner)
    {
      auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
      if (params.profiles)
      {
        if (params.profiles->hasProfileEntry<SimplePlannerPlanProfile>())
          interpolator->plan_profiles = params.profiles->getProfileEntry<SimplePlannerPlanProfile>();

        if (params.profiles->hasProfileEntry<SimplePlannerCompositeProfile>())
          interpolator->composite_profiles = params.profiles->getProfileEntry<SimplePlannerCompositeProfile>();
      }
      auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
      interpolator_idx = generator_->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }

    // Setup Seed Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
    // to short.
    auto seed_min_length_generator = std::make_unique<SeedMinLengthProcessGenerator>();
    int seed_min_length_idx =
        generator_->addNode(std::move(seed_min_length_generator), GraphTaskflow::NodeType::CONDITIONAL);

    // Setup TrajOpt
    auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
    trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
    if (params.profiles)
    {
      if (params.profiles->hasProfileEntry<TrajOptPlanProfile>())
        trajopt_planner->plan_profiles = params.profiles->getProfileEntry<TrajOptPlanProfile>();

      if (params.profiles->hasProfileEntry<TrajOptCompositeProfile>())
        trajopt_planner->composite_profiles = params.profiles->getProfileEntry<TrajOptCompositeProfile>();
    }
    auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
    int trajopt_idx = generator_->addNode(std::move(trajopt_generator), GraphTaskflow::NodeType::CONDITIONAL);

    // Setup OMPL
    auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
    ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
    if (params.profiles)
    {
      if (params.profiles->hasProfileEntry<OMPLPlanProfile>())
        ompl_planner->plan_profiles = params.profiles->getProfileEntry<OMPLPlanProfile>();
    }
    auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
    int ompl_idx = generator_->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::CONDITIONAL);

    // Setup TrajOpt 2
    auto trajopt_planner2 = std::make_shared<TrajOptMotionPlanner>();
    trajopt_planner2->problem_generator = &DefaultTrajoptProblemGenerator;
    if (params.profiles)
    {
      if (params.profiles->hasProfileEntry<TrajOptPlanProfile>())
        trajopt_planner2->plan_profiles = params.profiles->getProfileEntry<TrajOptPlanProfile>();

      if (params.profiles->hasProfileEntry<TrajOptCompositeProfile>())
        trajopt_planner2->composite_profiles = params.profiles->getProfileEntry<TrajOptCompositeProfile>();
    }
    auto trajopt_generator2 = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner2);
    int trajopt_idx2 = generator_->addNode(std::move(trajopt_generator2), GraphTaskflow::NodeType::CONDITIONAL);

    // Add Final Continuous Contact Check of trajectory
    int contact_check_idx{ -1 };
    if (params.enable_post_contact_continuous_check)
    {
      auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
      contact_check_idx = generator_->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }
    else if (params.enable_post_contact_discrete_check)
    {
      auto contact_check_generator = std::make_unique<DiscreteContactCheckProcessGenerator>();
      contact_check_idx = generator_->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }

    // Time parameterization trajectory
    int time_parameterization_idx{ -1 };
    if (params.enable_time_parameterization)
    {
      auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
      time_parameterization_idx =
          generator_->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);
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
      generator_->addEdge(interpolator_idx, ON_SUCCESS, seed_min_length_idx, PROCESS_NODE);
      generator_->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    }

    generator_->addEdge(seed_min_length_idx, ON_SUCCESS, trajopt_idx, PROCESS_NODE);
    generator_->addEdge(seed_min_length_idx, ON_FAILURE, -1, ERROR_CALLBACK);

    generator_->addEdge(trajopt_idx, ON_FAILURE, ompl_idx, PROCESS_NODE);
    if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
      generator_->addEdge(trajopt_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
    else if (params.enable_time_parameterization)
      generator_->addEdge(trajopt_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
    else
      generator_->addEdge(trajopt_idx, ON_SUCCESS, -1, DONE_CALLBACK);

    generator_->addEdge(ompl_idx, ON_SUCCESS, trajopt_idx2, PROCESS_NODE);
    generator_->addEdge(ompl_idx, ON_FAILURE, -1, ERROR_CALLBACK);

    generator_->addEdge(trajopt_idx2, ON_FAILURE, -1, ERROR_CALLBACK);
    if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
      generator_->addEdge(trajopt_idx2, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
    else if (params.enable_time_parameterization)
      generator_->addEdge(trajopt_idx2, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
    else
      generator_->addEdge(trajopt_idx2, ON_SUCCESS, -1, DONE_CALLBACK);

    if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    {
      generator_->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);
      if (params.enable_time_parameterization)
        generator_->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
      else
        generator_->addEdge(contact_check_idx, ON_SUCCESS, -1, DONE_CALLBACK);
    }

    if (params.enable_time_parameterization)
    {
      generator_->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
      generator_->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    }
  }
  else
  {
    // Setup Interpolator
    int interpolator_idx{ -1 };
    if (params.enable_simple_planner)
    {
      auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
      if (params.profiles)
      {
        if (params.profiles->hasProfileEntry<SimplePlannerPlanProfile>())
          interpolator->plan_profiles = params.profiles->getProfileEntry<SimplePlannerPlanProfile>();

        if (params.profiles->hasProfileEntry<SimplePlannerCompositeProfile>())
          interpolator->composite_profiles = params.profiles->getProfileEntry<SimplePlannerCompositeProfile>();
      }
      auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
      interpolator_idx = generator_->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }

    // Setup Seed Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
    // to short.
    auto seed_min_length_generator = std::make_unique<SeedMinLengthProcessGenerator>();
    int seed_min_length_idx =
        generator_->addNode(std::move(seed_min_length_generator), GraphTaskflow::NodeType::CONDITIONAL);

    // Setup OMPL
    auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
    ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
    if (params.profiles)
    {
      if (params.profiles->hasProfileEntry<OMPLPlanProfile>())
        ompl_planner->plan_profiles = params.profiles->getProfileEntry<OMPLPlanProfile>();
    }
    auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
    int ompl_idx = generator_->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::TASK);

    // Setup TrajOpt
    auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
    trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
    if (params.profiles)
    {
      if (params.profiles->hasProfileEntry<TrajOptPlanProfile>())
        trajopt_planner->plan_profiles = params.profiles->getProfileEntry<TrajOptPlanProfile>();

      if (params.profiles->hasProfileEntry<TrajOptCompositeProfile>())
        trajopt_planner->composite_profiles = params.profiles->getProfileEntry<TrajOptCompositeProfile>();
    }
    auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
    int trajopt_idx = generator_->addNode(std::move(trajopt_generator), GraphTaskflow::NodeType::CONDITIONAL);

    // Add Final Continuous Contact Check of trajectory
    int contact_check_idx{ -1 };
    if (params.enable_post_contact_continuous_check)
    {
      auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
      contact_check_idx = generator_->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }
    else if (params.enable_post_contact_discrete_check)
    {
      auto contact_check_generator = std::make_unique<DiscreteContactCheckProcessGenerator>();
      contact_check_idx = generator_->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);
    }

    // Time parameterization trajectory
    int time_parameterization_idx{ -1 };
    if (params.enable_time_parameterization)
    {
      auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
      time_parameterization_idx =
          generator_->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);
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
      generator_->addEdge(interpolator_idx, ON_SUCCESS, seed_min_length_idx, PROCESS_NODE);
      generator_->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    }

    generator_->addEdge(seed_min_length_idx, ON_SUCCESS, ompl_idx, PROCESS_NODE);
    generator_->addEdge(seed_min_length_idx, ON_FAILURE, -1, ERROR_CALLBACK);

    generator_->addEdge(ompl_idx, ON_NONE, trajopt_idx, PROCESS_NODE);

    generator_->addEdge(trajopt_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
      generator_->addEdge(trajopt_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
    else if (params.enable_time_parameterization)
      generator_->addEdge(trajopt_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
    else
      generator_->addEdge(trajopt_idx, ON_SUCCESS, -1, DONE_CALLBACK);

    if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    {
      generator_->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);
      if (params.enable_time_parameterization)
        generator_->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
      else
        generator_->addEdge(contact_check_idx, ON_SUCCESS, -1, DONE_CALLBACK);
    }

    if (params.enable_time_parameterization)
    {
      generator_->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
      generator_->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);
    }
  }
}

const std::string& FreespaceTaskflow::getName() const { return name_; }

tf::Taskflow& FreespaceTaskflow::generateTaskflow(ProcessInput input,
                                                  std::function<void()> done_cb,
                                                  std::function<void()> error_cb)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    throw std::runtime_error("Invalid Process Input");
  }

  return generator_->generateTaskflow(input, done_cb, error_cb);
}

void FreespaceTaskflow::abort() { generator_->abort(); }

void FreespaceTaskflow::reset() { generator_->reset(); }

void FreespaceTaskflow::clear() { generator_->clear(); }

bool FreespaceTaskflow::checkProcessInput(const tesseract_planning::ProcessInput& input) const
{
  // Check Input
  if (!input.tesseract)
  {
    CONSOLE_BRIDGE_logError("ProcessInput tesseract is a nullptr");
    return false;
  }

  // Check the overall input
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should be a composite");
    return false;
  }

  return true;
}

void FreespaceTaskflow::successCallback(std::function<void()> user_callback)
{
  CONSOLE_BRIDGE_logInform("%s Successful", name_.c_str());
  if (user_callback)
    user_callback();
}

void FreespaceTaskflow::failureCallback(std::function<void()> user_callback)
{
  // For this process, any failure of a sub-TaskFlow indicates a planning failure. Abort all future tasks
  generator_->abort();

  // Print an error if this is the first failure
  CONSOLE_BRIDGE_logError("%s Failure", name_.c_str());
  if (user_callback)
    user_callback();
}
