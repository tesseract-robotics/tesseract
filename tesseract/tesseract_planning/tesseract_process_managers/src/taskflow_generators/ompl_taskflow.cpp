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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/taskflow_generators/ompl_taskflow.h>

#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/continuous_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>

using namespace tesseract_planning;

OMPLTaskflow::OMPLTaskflow(OMPLTaskflowParams params, std::string name) : name_(name), params_(params) {}

const std::string& OMPLTaskflow::getName() const { return name_; }

TaskflowContainer OMPLTaskflow::generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb)
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
  tf::Task ompl_task = container.taskflow->placeholder();

  has_seed_task.precede(interpolator_task, ompl_task);
  interpolator_task.precede(error_task, ompl_task);

  // Setup Interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>("Interpolator");
  if (input.profiles)
  {
    if (input.profiles->hasProfileEntry<SimplePlannerPlanProfile>())
      interpolator->plan_profiles = input.profiles->getProfileEntry<SimplePlannerPlanProfile>();

    if (input.profiles->hasProfileEntry<SimplePlannerCompositeProfile>())
      interpolator->composite_profiles = input.profiles->getProfileEntry<SimplePlannerCompositeProfile>();
  }
  auto interpolator_generator = std::make_unique<MotionPlannerTaskGenerator>(interpolator);
  interpolator_generator->assignConditionalTask(input, interpolator_task);
  container.generators.push_back(std::move(interpolator_generator));

  // Setup TrajOpt
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  if (input.profiles)
  {
    if (input.profiles->hasProfileEntry<OMPLPlanProfile>())
      ompl_planner->plan_profiles = input.profiles->getProfileEntry<OMPLPlanProfile>();
  }
  auto ompl_generator = std::make_unique<MotionPlannerTaskGenerator>(ompl_planner);
  ompl_generator->assignConditionalTask(input, ompl_task);
  container.generators.push_back(std::move(ompl_generator));

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
    ompl_task.precede(error_task, contact_task);
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
    ompl_task.precede(error_task, contact_task);
    contact_task.precede(error_task, done_task);
    container.generators.push_back(std::move(contact_check_generator));
  }
  else if (!has_contact_check && params_.enable_time_parameterization)
  {
    tf::Task time_task = container.taskflow->placeholder();
    time_parameterization_generator->assignConditionalTask(input, time_task);
    container.generators.push_back(std::move(time_parameterization_generator));
    ompl_task.precede(error_task, time_task);
    time_task.precede(error_task, done_task);
    container.generators.push_back(std::move(time_parameterization_generator));
  }
  else
  {
    ompl_task.precede(error_task, done_task);
  }

  return container;
}

bool OMPLTaskflow::checkTaskInput(const tesseract_planning::TaskInput& input) const
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
