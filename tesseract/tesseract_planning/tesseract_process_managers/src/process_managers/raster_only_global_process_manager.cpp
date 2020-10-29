/**
 * @file raster_only_global_process_manager.cpp
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

#include <tesseract_process_managers/process_managers/raster_only_global_process_manager.h>
#include <tesseract_process_managers/debug_observer.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

RasterOnlyGlobalProcessManager::RasterOnlyGlobalProcessManager(TaskflowGenerator::UPtr global_taskflow_generator,
                                                               TaskflowGenerator::UPtr transition_taskflow_generator,
                                                               TaskflowGenerator::UPtr raster_taskflow_generator,
                                                               std::size_t n)
  : global_taskflow_generator_(std::move(global_taskflow_generator))
  , transition_taskflow_generator_(std::move(transition_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , executor_(n)
  , taskflow_("RasterOnlyGlobalProcessManagerTaskflow")
{
}

bool RasterOnlyGlobalProcessManager::init(ProcessInput input)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    return false;
  }

  // Clear the process manager
  clear();

  // Store the current size of the tasks so that we can add from_start later
  std::size_t starting_raster_idx = raster_tasks_.size();

  const Instruction* input_instruction = input.getInstruction();
  global_task_ =
      taskflow_
          .composed_of(global_taskflow_generator_->generateTaskflow(
              input,
              std::bind(&RasterOnlyGlobalProcessManager::successCallback, this, input_instruction->getDescription()),
              std::bind(&RasterOnlyGlobalProcessManager::failureCallback, this, input_instruction->getDescription())))
          .name("global");

  global_post_task_ = taskflow_.emplace([input]() { globalPostProcess(input); }).name("global post process");
  global_task_.precede(global_post_task_);

  // Generate all of the raster tasks. They don't depend on anything
  for (std::size_t idx = 0; idx < input.size(); idx += 2)
  {
    ProcessInput raster_input = input[idx];
    if (idx == 0)
      raster_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()->getStartInstruction());
    else
      raster_input.setStartInstruction(std::vector<std::size_t>({ idx - 1 }));

    if (idx < (input.size() - 1))
      raster_input.setEndInstruction(std::vector<std::size_t>({ idx + 1 }));

    auto raster_step =
        taskflow_
            .composed_of(raster_taskflow_generator_->generateTaskflow(
                raster_input,
                std::bind(&RasterOnlyGlobalProcessManager::successCallback,
                          this,
                          raster_input.getInstruction()->getDescription()),
                std::bind(&RasterOnlyGlobalProcessManager::failureCallback,
                          this,
                          raster_input.getInstruction()->getDescription())))
            .name("Raster #" + std::to_string(idx / 2) + ": " + raster_input.getInstruction()->getDescription());
    global_post_task_.precede(raster_step);
    raster_tasks_.push_back(raster_step);
  }

  // Loop over all transitions
  std::size_t transition_idx = 0;
  for (std::size_t input_idx = 1; input_idx < input.size() - 1; input_idx += 2)
  {
    // This use to extract the start and end, but things were changed so the seed is generated as part of the
    // taskflow. So the seed is only a skeleton and does not contain move instructions. So instead we provide the
    // composite and let the generateTaskflow extract the start and end waypoint from the composite. This is also more
    // robust because planners could modify composite size, which is rare but does happen when using OMPL where it is
    // not possible to simplify the trajectory to the desired number of states.
    ProcessInput transition_input = input[input_idx];
    transition_input.setStartInstruction(std::vector<std::size_t>({ input_idx - 1 }));
    transition_input.setEndInstruction(std::vector<std::size_t>({ input_idx + 1 }));
    auto transition_step = taskflow_
                               .composed_of(transition_taskflow_generator_->generateTaskflow(
                                   transition_input,
                                   std::bind(&RasterOnlyGlobalProcessManager::successCallback,
                                             this,
                                             transition_input.getInstruction()->getDescription()),
                                   std::bind(&RasterOnlyGlobalProcessManager::failureCallback,
                                             this,
                                             transition_input.getInstruction()->getDescription())))
                               .name("Transition #" + std::to_string(transition_idx) + ": " +
                                     transition_input.getInstruction()->getDescription());

    // Each transition is independent and thus depends only on the adjacent rasters
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx]);
    transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx + 1]);

    transition_tasks_.push_back(transition_step);
    transition_idx++;
  }

  // visualizes the taskflow
  std::ofstream out_data;
  out_data.open(tesseract_common::getTempPath() + "raster_only_global_process_manager.dot");
  taskflow_.dump(out_data);
  out_data.close();

  return true;
}

bool RasterOnlyGlobalProcessManager::execute()
{
  success_ = true;

  DebugObserver::Ptr debug_observer;
  std::shared_ptr<tf::TFProfObserver> profile_observer;
  if (debug_)
    debug_observer = executor_.make_observer<DebugObserver>("RasterOnlyGlobalProcessManagerObserver");

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
    out_data.open(tesseract_common::getTempPath() + "raster_only_global_process_manager_profile-" +
                  tesseract_common::getTimestampString() + ".json");
    profile_observer->dump(out_data);
    out_data.close();
    executor_.remove_observer(profile_observer);
  }

  clear();  // I believe clear must be called so memory is cleaned up

  return success_;
}

bool RasterOnlyGlobalProcessManager::terminate()
{
  global_taskflow_generator_->abort();
  transition_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();

  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool RasterOnlyGlobalProcessManager::clear()

{
  global_taskflow_generator_->clear();
  global_task_.reset();
  global_post_task_.reset();
  transition_taskflow_generator_->clear();
  raster_taskflow_generator_->clear();
  taskflow_.clear();
  raster_tasks_.clear();

  return true;
}

void RasterOnlyGlobalProcessManager::enableDebug(bool enabled) { debug_ = enabled; }

void RasterOnlyGlobalProcessManager::enableProfile(bool enabled) { profile_ = enabled; }

void RasterOnlyGlobalProcessManager::globalPostProcess(ProcessInput input)
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

bool RasterOnlyGlobalProcessManager::checkProcessInput(const tesseract_planning::ProcessInput& input) const
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

  // Check rasters and transitions
  for (std::size_t index = 0; index < composite->size(); index++)
  {
    // Both rasters and transitions should be a composite
    if (!isCompositeInstruction(composite->at(index)))
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: Both rasters and transitions should be a composite");
      return false;
    }
  }

  return true;
}

void RasterOnlyGlobalProcessManager::successCallback(std::string message)
{
  CONSOLE_BRIDGE_logInform("%s", message.c_str());
  success_ &= true;
}

void RasterOnlyGlobalProcessManager::failureCallback(std::string message)
{
  // For this process, any failure of a sub-TaskFlow indicates a planning failure. Abort all future tasks
  global_taskflow_generator_->abort();
  transition_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();

  // Print an error if this is the first failure
  if (success_)
    CONSOLE_BRIDGE_logError("RasterOnlyGlobalProcessManager Failure: %s", message.c_str());
  success_ = false;
}
