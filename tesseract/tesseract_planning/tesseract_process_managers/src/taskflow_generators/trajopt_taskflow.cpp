/**
 * @file trajopt_taskflow.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>

#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/continuous_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

using namespace tesseract_planning;

TrajOptTaskflow::TrajOptTaskflow(TrajOptTaskflowParams params, std::string name) : name_(name), params_(params) {}

const std::string& TrajOptTaskflow::getName() const { return name_; }

TaskflowContainer TrajOptTaskflow::generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkTaskInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    throw std::runtime_error("Invalid Process Input");
  }

  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);

  // Add "Error" task
  auto error_fn = [=]() { failureTask(input, name_, "", error_cb); };
  tf::Task error_task = container.taskflow->emplace(error_fn).name("Error Callback");
  container.outputs.push_back(error_task);

  // Add "Done" task
  auto done_fn = [=]() { successTask(input, name_, "", done_cb); };
  tf::Task done_task = container.taskflow->emplace(done_fn).name("Done Callback");
  container.outputs.push_back(done_task);

  // Add has seed check
  tf::Task has_seed_task = container.taskflow->emplace([=]() { return hasSeedTask(input); }).name("Has Seed Check");

  tf::Task interpolator_task = container.taskflow->placeholder();
  tf::Task seed_min_length_task = container.taskflow->placeholder();
  tf::Task trajopt_task = container.taskflow->placeholder();

  has_seed_task.precede(interpolator_task, seed_min_length_task);
  interpolator_task.precede(error_task, seed_min_length_task);
  seed_min_length_task.precede(trajopt_task);

  // Setup Interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
  if (input.profiles)
  {
    if (input.profiles->hasProfileEntry<SimplePlannerPlanProfile>())
      interpolator->plan_profiles = input.profiles->getProfileEntry<SimplePlannerPlanProfile>();

    if (input.profiles->hasProfileEntry<SimplePlannerCompositeProfile>())
      interpolator->composite_profiles = input.profiles->getProfileEntry<SimplePlannerCompositeProfile>();
  }
  TaskGenerator::UPtr interpolator_generator = std::make_unique<MotionPlannerTaskGenerator>(interpolator);
  interpolator_generator->assignConditionalTask(input, interpolator_task);
  container.generators.push_back(std::move(interpolator_generator));

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  TaskGenerator::UPtr seed_min_length_generator = std::make_unique<SeedMinLengthTaskGenerator>();
  seed_min_length_generator->assignTask(input, seed_min_length_task);
  container.generators.push_back(std::move(seed_min_length_generator));

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
  if (input.profiles)
  {
    if (input.profiles->hasProfileEntry<TrajOptPlanProfile>())
      trajopt_planner->plan_profiles = input.profiles->getProfileEntry<TrajOptPlanProfile>();

    if (input.profiles->hasProfileEntry<TrajOptCompositeProfile>())
      trajopt_planner->composite_profiles = input.profiles->getProfileEntry<TrajOptCompositeProfile>();

    if (input.profiles->hasProfileEntry<TrajOptSolverProfile>())
      trajopt_planner->solver_profiles = input.profiles->getProfileEntry<TrajOptSolverProfile>();
  }
  TaskGenerator::UPtr trajopt_generator = std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner);
  trajopt_generator->assignConditionalTask(input, trajopt_task);
  container.generators.push_back(std::move(trajopt_generator));

  TaskGenerator::UPtr contact_check_generator;
  bool has_contact_check = (params_.enable_post_contact_continuous_check || params_.enable_post_contact_discrete_check);
  if (has_contact_check)
  {
    if (params_.enable_post_contact_continuous_check)
      contact_check_generator = std::make_unique<ContinuousContactCheckTaskGenerator>();
    else if (params_.enable_post_contact_discrete_check)
      contact_check_generator = std::make_unique<DiscreteContactCheckTaskGenerator>();
  }

  TaskGenerator::UPtr time_parameterization_generator;
  if (params_.enable_time_parameterization)
    time_parameterization_generator = std::make_unique<IterativeSplineParameterizationTaskGenerator>();

  // Add Final Continuous Contact Check of trajectory and Time parameterization trajectory
  if (has_contact_check && params_.enable_time_parameterization)
  {
    tf::Task contact_task = container.taskflow->placeholder();
    contact_check_generator->assignConditionalTask(input, contact_task);
    trajopt_task.precede(error_task, contact_task);
    container.generators.push_back(std::move(contact_check_generator));

    tf::Task time_task = container.taskflow->placeholder();
    time_parameterization_generator->assignConditionalTask(input, time_task);
    container.generators.push_back(std::move(time_parameterization_generator));
    contact_task.precede(error_task, time_task);
    time_task.precede(error_task, done_task);
    container.generators.push_back(std::move(time_parameterization_generator));
  }
  else if (has_contact_check && !params_.enable_time_parameterization)
  {
    tf::Task contact_task = container.taskflow->placeholder();
    contact_check_generator->assignConditionalTask(input, contact_task);
    trajopt_task.precede(error_task, contact_task);
    contact_task.precede(error_task, done_task);
    container.generators.push_back(std::move(contact_check_generator));
  }
  else if (!has_contact_check && params_.enable_time_parameterization)
  {
    tf::Task time_task = container.taskflow->placeholder();
    time_parameterization_generator->assignConditionalTask(input, time_task);
    container.generators.push_back(std::move(time_parameterization_generator));
    trajopt_task.precede(error_task, time_task);
    time_task.precede(error_task, done_task);
    container.generators.push_back(std::move(time_parameterization_generator));
  }
  else
  {
    trajopt_task.precede(error_task, done_task);
  }

  return container;
}

bool TrajOptTaskflow::checkTaskInput(const tesseract_planning::TaskInput& input) const
{
  // Check Input
  if (!input.env)
  {
    CONSOLE_BRIDGE_logError("TaskInput env is a nullptr");
    return false;
  }

  // Check the overall input
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: input.instructions should be a composite");
    return false;
  }

  return true;
}
