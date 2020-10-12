/**
 * @file raster_process_manager.cpp
 * @brief Plans raster paths
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/debug_observer.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

RasterProcessManager::RasterProcessManager(TaskflowGenerator::UPtr freespace_taskflow_generator,
                                           TaskflowGenerator::UPtr transition_taskflow_generator,
                                           TaskflowGenerator::UPtr raster_taskflow_generator,
                                           std::size_t n)
  : freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , transition_taskflow_generator_(std::move(transition_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , executor_(n)
  , taskflow_("RasterProcessManagerTaskflow")
{
}

RasterProcessManager::RasterProcessManager(TaskflowGenerator::UPtr global_taskflow_generator,
                                           TaskflowGenerator::UPtr freespace_taskflow_generator,
                                           TaskflowGenerator::UPtr transition_taskflow_generator,
                                           TaskflowGenerator::UPtr raster_taskflow_generator,
                                           std::size_t n)
  : global_taskflow_generator_(std::move(global_taskflow_generator))
  , freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , transition_taskflow_generator_(std::move(transition_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , executor_(n)
  , taskflow_("RasterGlobalProcessManagerTaskflow")
{
}

bool RasterProcessManager::init(ProcessInput input)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    return false;
  }

  // Clear the process manager
  clear();

  if (global_taskflow_generator_ != nullptr)
    return initGlobal(input);

  return initDefault(input);
}

bool RasterProcessManager::initGlobal(ProcessInput input)
{
  // Store the current size of the tasks so that we can add from_start later
  std::size_t starting_raster_idx = raster_tasks_.size();

  const Instruction* input_instruction = input.getInstruction();
  global_task_ = taskflow_
                     .composed_of(global_taskflow_generator_->generateTaskflow(
                         input,
                         std::bind(&RasterProcessManager::successCallback, this, input_instruction->getDescription()),
                         std::bind(&RasterProcessManager::failureCallback, this, input_instruction->getDescription())))
                     .name("global");

  global_post_task_ = taskflow_.emplace([input]() { globalPostProcess(input); }).name("global post process");
  global_task_.precede(global_post_task_);

  // Generate all of the raster tasks. They don't depend on anything
  for (std::size_t idx = 1; idx < input.size() - 1; idx += 2)
  {
    ProcessInput raster_input = input[idx];
    raster_input.setStartInstruction(std::vector<std::size_t>({ idx - 1 }));
    raster_input.setEndInstruction(std::vector<std::size_t>({ idx + 1 }));
    auto raster_step =
        taskflow_
            .composed_of(raster_taskflow_generator_->generateTaskflow(
                raster_input,
                std::bind(
                    &RasterProcessManager::successCallback, this, raster_input.getInstruction()->getDescription()),
                std::bind(
                    &RasterProcessManager::failureCallback, this, raster_input.getInstruction()->getDescription())))
            .name("Raster #" + std::to_string((idx - 1) / 2) + ": " + raster_input.getInstruction()->getDescription());
    global_post_task_.precede(raster_step);
    raster_tasks_.push_back(raster_step);
  }

  // Loop over all transitions
  std::size_t transition_idx = 0;
  for (std::size_t input_idx = 2; input_idx < input.size() - 2; input_idx += 2)
  {
    // This use to extract the start and end, but things were changed so the seed is generated as part of the
    // taskflow. So the seed is only a skeleton and does not contain move instructions. So instead we provide the
    // composite and let the generateTaskflow extract the start and end waypoint from the composite. This is also more
    // robust because planners could modify composite size, which is rare but does happen when using OMPL where it is
    // not possible to simplify the trajectory to the desired number of states.
    ProcessInput transition_input = input[input_idx];
    transition_input.setStartInstruction(std::vector<std::size_t>({ input_idx - 1 }));
    transition_input.setEndInstruction(std::vector<std::size_t>({ input_idx + 1 }));
    auto transition_step =
        taskflow_
            .composed_of(transition_taskflow_generator_->generateTaskflow(
                transition_input,
                std::bind(
                    &RasterProcessManager::successCallback, this, transition_input.getInstruction()->getDescription()),
                std::bind(
                    &RasterProcessManager::failureCallback, this, transition_input.getInstruction()->getDescription())))
            .name("Transition #" + std::to_string(transition_idx) + ": " +
                  transition_input.getInstruction()->getDescription());

    // Each transition is independent and thus depends only on the adjacent rasters
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx]);
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx + 1]);

    transition_tasks_.push_back(transition_step);
    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  ProcessInput from_start_input = input[0];
  from_start_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()->getStartInstruction());
  from_start_input.setEndInstruction(std::vector<std::size_t>({ 1 }));
  auto from_start =
      taskflow_
          .composed_of(freespace_taskflow_generator_->generateTaskflow(
              from_start_input,
              std::bind(
                  &RasterProcessManager::successCallback, this, from_start_input.getInstruction()->getDescription()),
              std::bind(
                  &RasterProcessManager::failureCallback, this, from_start_input.getInstruction()->getDescription())))
          .name("From Start: " + from_start_input.getInstruction()->getDescription());
  raster_tasks_[starting_raster_idx].precede(from_start);
  freespace_tasks_.push_back(from_start);

  // Plan to_end - preceded by the last raster
  ProcessInput to_end_input = input[input.size() - 1];
  to_end_input.setStartInstruction(std::vector<std::size_t>({ input.size() - 2 }));

  auto to_end =
      taskflow_
          .composed_of(freespace_taskflow_generator_->generateTaskflow(
              to_end_input,
              std::bind(&RasterProcessManager::successCallback, this, to_end_input.getInstruction()->getDescription()),
              std::bind(&RasterProcessManager::failureCallback, this, to_end_input.getInstruction()->getDescription())))
          .name("To End: " + to_end_input.getInstruction()->getDescription());
  raster_tasks_.back().precede(to_end);
  freespace_tasks_.push_back(to_end);

  // visualizes the taskflow
  std::ofstream out_data;
  out_data.open(tesseract_common::getTempPath() + "raster_global_process_manager.dot");
  taskflow_.dump(out_data);
  out_data.close();

  return true;
}

bool RasterProcessManager::initDefault(ProcessInput input)
{
  // Store the current size of the tasks so that we can add from_start later
  std::size_t starting_raster_idx = raster_tasks_.size();

  // Generate all of the raster tasks. They don't depend on anything
  const Instruction* input_instruction = input.getInstruction();
  for (std::size_t idx = 1; idx < input.size() - 1; idx += 2)
  {
    // Get Start Plan Instruction
    Instruction start_instruction = NullInstruction();
    if (idx == 1)
    {
      ProcessInput from_start_input = input[0];
      const Instruction* from_start_input_instruction = from_start_input.getInstruction();
      assert(isCompositeInstruction(*from_start_input_instruction));
      const auto* ci = from_start_input_instruction->cast_const<CompositeInstruction>();
      const auto* li = getLastPlanInstruction(*ci);
      assert(li != nullptr);
      start_instruction = *li;
    }
    else
    {
      ProcessInput pre_input = input[idx - 1];
      const Instruction* pre_input_instruction = pre_input.getInstruction();
      assert(isCompositeInstruction(*pre_input_instruction));
      const auto* tci = pre_input_instruction->cast_const<CompositeInstruction>();
      auto* li = getLastPlanInstruction(*tci);
      assert(li != nullptr);
      start_instruction = *li;
    }

    start_instruction.cast<PlanInstruction>()->setPlanType(PlanInstructionType::START);
    ProcessInput raster_input = input[idx];
    raster_input.setStartInstruction(start_instruction);
    auto raster_step =
        taskflow_
            .composed_of(raster_taskflow_generator_->generateTaskflow(
                raster_input,
                std::bind(
                    &RasterProcessManager::successCallback, this, raster_input.getInstruction()->getDescription()),
                std::bind(
                    &RasterProcessManager::failureCallback, this, raster_input.getInstruction()->getDescription())))
            .name("Raster #" + std::to_string((idx - 1) / 2) + ": " + raster_input.getInstruction()->getDescription());
    raster_tasks_.push_back(raster_step);
  }

  // Loop over all transitions
  std::size_t transition_idx = 0;
  for (std::size_t input_idx = 2; input_idx < input.size() - 2; input_idx += 2)
  {
    // This use to extract the start and end, but things were changed so the seed is generated as part of the
    // taskflow. So the seed is only a skeleton and does not contain move instructions. So instead we provide the
    // composite and let the generateTaskflow extract the start and end waypoint from the composite. This is also more
    // robust because planners could modify composite size, which is rare but does happen when using OMPL where it is
    // not possible to simplify the trajectory to the desired number of states.
    ProcessInput transition_input = input[input_idx];
    transition_input.setStartInstruction(std::vector<std::size_t>({ input_idx - 1 }));
    transition_input.setEndInstruction(std::vector<std::size_t>({ input_idx + 1 }));
    auto transition_step =
        taskflow_
            .composed_of(transition_taskflow_generator_->generateTaskflow(
                transition_input,
                std::bind(
                    &RasterProcessManager::successCallback, this, transition_input.getInstruction()->getDescription()),
                std::bind(
                    &RasterProcessManager::failureCallback, this, transition_input.getInstruction()->getDescription())))
            .name("Transition #" + std::to_string(transition_idx) + ": " +
                  transition_input.getInstruction()->getDescription());

    // Each transition is independent and thus depends only on the adjacent rasters
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx]);
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx + 1]);

    transition_tasks_.push_back(transition_step);
    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  ProcessInput from_start_input = input[0];
  from_start_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()->getStartInstruction());
  from_start_input.setEndInstruction(std::vector<std::size_t>({ 1 }));

  auto from_start =
      taskflow_
          .composed_of(freespace_taskflow_generator_->generateTaskflow(
              from_start_input,
              std::bind(
                  &RasterProcessManager::successCallback, this, from_start_input.getInstruction()->getDescription()),
              std::bind(
                  &RasterProcessManager::failureCallback, this, from_start_input.getInstruction()->getDescription())))
          .name("From Start: " + from_start_input.getInstruction()->getDescription());
  raster_tasks_[starting_raster_idx].precede(from_start);
  freespace_tasks_.push_back(from_start);

  // Plan to_end - preceded by the last raster
  ProcessInput to_end_input = input[input.size() - 1];
  to_end_input.setStartInstruction(std::vector<std::size_t>({ input.size() - 2 }));

  auto to_end =
      taskflow_
          .composed_of(freespace_taskflow_generator_->generateTaskflow(
              to_end_input,
              std::bind(&RasterProcessManager::successCallback, this, to_end_input.getInstruction()->getDescription()),
              std::bind(&RasterProcessManager::failureCallback, this, to_end_input.getInstruction()->getDescription())))
          .name("To End: " + to_end_input.getInstruction()->getDescription());
  raster_tasks_.back().precede(to_end);
  freespace_tasks_.push_back(to_end);

  // visualizes the taskflow
  if (debug_)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + "raster_process_manager-" + tesseract_common::getTimestampString() +
                  ".dot");
    taskflow_.dump(out_data);
    out_data.close();
  }

  return true;
}

bool RasterProcessManager::execute()
{
  success_ = true;

  DebugObserver::Ptr debug_observer;
  std::shared_ptr<tf::TFProfObserver> profile_observer;
  if (debug_)
    debug_observer = executor_.make_observer<DebugObserver>("RasterProcessManagerObserver");

  if (profile_)
    profile_observer = executor_.make_observer<tf::TFProfObserver>();

  // TODO: Figure out how to cancel execution. This callback is only checked at beginning of the taskflow (ie before
  // restarting)
  //  executor.run_until(taskflow, [this]() { std::cout << "Checking if done: " << this->done << std::endl; return
  //  this->done;});

  // Wait for currently running taskflows to end.
  executor_.wait_for_all();
  executor_.run(taskflow_);
  executor_.wait_for_all();

  if (debug_observer != nullptr)
    executor_.remove_observer(debug_observer);

  if (profile_observer != nullptr)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + "raster_process_manager_profile-" +
                  tesseract_common::getTimestampString() + ".json");
    profile_observer->dump(out_data);
    out_data.close();
    executor_.remove_observer(profile_observer);
  }

  clear();  // I believe clear must be called so memory is cleaned up

  return success_;
}

bool RasterProcessManager::terminate()
{
  if (global_taskflow_generator_ != nullptr)
    global_taskflow_generator_->abort();
  freespace_taskflow_generator_->abort();
  transition_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();

  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool RasterProcessManager::clear()

{
  if (global_taskflow_generator_ != nullptr)
    global_taskflow_generator_->clear();

  global_task_.reset();
  global_post_task_.reset();
  freespace_taskflow_generator_->clear();
  transition_taskflow_generator_->clear();
  raster_taskflow_generator_->clear();
  taskflow_.clear();
  freespace_tasks_.clear();
  raster_tasks_.clear();

  return true;
}

void RasterProcessManager::enableDebug(bool enabled) { debug_ = enabled; }

void RasterProcessManager::enableProfile(bool enabled) { profile_ = enabled; }

void RasterProcessManager::globalPostProcess(ProcessInput input)
{
  CompositeInstruction* results = input.getResults()->cast<CompositeInstruction>();
  CompositeInstruction* composite = results->at(0).cast<CompositeInstruction>();
  composite->setStartInstruction(results->getStartInstruction());
  composite->setManipulatorInfo(results->getManipulatorInfo());
  for (std::size_t i = 1; i < results->size(); ++i)
  {
    CompositeInstruction* composite0 = results->at(i - 1).cast<CompositeInstruction>();
    MoveInstruction lmi = *getLastMoveInstruction(*composite0);
    lmi.setMoveType(MoveInstructionType::START);

    CompositeInstruction* composite1 = results->at(i).cast<CompositeInstruction>();
    composite1->setStartInstruction(lmi);
    composite1->setManipulatorInfo(results->getManipulatorInfo());
  }
}

bool RasterProcessManager::checkProcessInput(const tesseract_planning::ProcessInput& input) const
{
  // -------------
  // Check Input
  // -------------
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
  const auto* composite = input_instruction->cast_const<CompositeInstruction>();

  // Check that it has a start instruction
  if (!composite->hasStartInstruction() && isNullInstruction(input.getStartInstruction()))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should have a start instruction");
    return false;
  }

  // Check from_start
  if (!isCompositeInstruction(composite->at(0)))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: from_start should be a composite");
    return false;
  }

  // Check rasters and transitions
  for (std::size_t index = 1; index < composite->size() - 1; index++)
  {
    // Both rasters and transitions should be a composite
    if (!isCompositeInstruction(composite->at(index)))
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: Both rasters and transitions should be a composite");
      return false;
    }
  }

  // Check to_end
  if (!isCompositeInstruction(composite->back()))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: to_end should be a composite");
    return false;
  };

  return true;
}

void RasterProcessManager::successCallback(std::string message)
{
  CONSOLE_BRIDGE_logInform("%s", message.c_str());
  success_ &= true;
}

void RasterProcessManager::failureCallback(std::string message)
{
  // For this process, any failure of a sub-TaskFlow indicates a planning failure. Abort all future tasks
  if (global_taskflow_generator_ != nullptr)
    global_taskflow_generator_->abort();

  freespace_taskflow_generator_->abort();
  transition_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();
  // Print an error if this is the first failure
  if (success_)
    CONSOLE_BRIDGE_logError("RasterProcessManager Failure: %s", message.c_str());
  success_ = false;
}
