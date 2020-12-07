/**
 * @file descartes_taskflow.cpp
 * @brief Descartes Graph Taskflow
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

#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/discrete_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

using namespace tesseract_planning;

DescartesTaskflow::DescartesTaskflow(DescartesTaskflowParams params, std::string name)
  : name_(name), params_(params), generator_(std::make_unique<GraphTaskflow>(name_))
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

  // Setup Descartes
  auto descartes_planner = std::make_shared<DescartesMotionPlanner<double>>();
  descartes_planner->problem_generator = &DefaultDescartesProblemGenerator<double>;
  if (params.profiles)
  {
    if (params.profiles->hasProfileEntry<DescartesPlanProfile<double>>())
      descartes_planner->plan_profiles = params.profiles->getProfileEntry<DescartesPlanProfile<double>>();
  }
  auto descartes_generator = std::make_unique<MotionPlannerProcessGenerator>(descartes_planner);
  int descartes_idx = generator_->addNode(std::move(descartes_generator), GraphTaskflow::NodeType::CONDITIONAL);

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
    generator_->addEdge(interpolator_idx, ON_SUCCESS, descartes_idx, PROCESS_NODE);
    generator_->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  }

  generator_->addEdge(descartes_idx, ON_FAILURE, -1, ERROR_CALLBACK);
  if (params.enable_post_contact_continuous_check || params.enable_post_contact_discrete_check)
    generator_->addEdge(descartes_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  else if (params.enable_time_parameterization)
    generator_->addEdge(descartes_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  else
    generator_->addEdge(descartes_idx, ON_SUCCESS, -1, DONE_CALLBACK);

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

const std::string& DescartesTaskflow::getName() const { return name_; }

tf::Taskflow& DescartesTaskflow::generateTaskflow(ProcessInput input,
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

void DescartesTaskflow::abort() { generator_->abort(); }

void DescartesTaskflow::reset() { generator_->reset(); }

void DescartesTaskflow::clear() { generator_->clear(); }

bool DescartesTaskflow::checkProcessInput(const tesseract_planning::ProcessInput& input) const
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

void DescartesTaskflow::successCallback(std::function<void()> user_callback)
{
  CONSOLE_BRIDGE_logInform("%s Successful", name_.c_str());
  if (user_callback)
    user_callback();
}

void DescartesTaskflow::failureCallback(std::function<void()> user_callback)
{
  // For this process, any failure of a sub-TaskFlow indicates a planning failure. Abort all future tasks
  generator_->abort();

  // Print an error if this is the first failure
  CONSOLE_BRIDGE_logError("%s Failure", name_.c_str());
  if (user_callback)
    user_callback();
}
