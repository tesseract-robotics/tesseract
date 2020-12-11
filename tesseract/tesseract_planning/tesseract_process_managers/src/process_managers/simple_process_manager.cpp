/**
 * @file simple_process_manager.cpp
 * @brief Plans simple paths aka. a single composite containing no composite instruction.
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#include <taskflow/taskflow.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_managers/simple_process_manager.h>
#include <tesseract_process_managers/core/debug_observer.h>
#include <tesseract_process_managers/core/taskflow_generator.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

SimpleProcessManager::SimpleProcessManager(TaskflowGenerator::UPtr taskflow_generator, std::size_t n)
  : taskflow_generator_(std::move(taskflow_generator)), executor_(n)
{
}

bool SimpleProcessManager::init(ProcessInput input)
{
  // Clear the process manager
  clear();

  process_input_ = std::make_shared<ProcessInput>(input);

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

  // Create the dependency graph
  if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
    input_instruction->print("Generating Taskflow for: ");
  taskflow_container_ = taskflow_generator_->generateTaskflow(
      input,
      [=]() { successTask(input, "SimpleProcessManager", input.getInstruction()->getDescription(), nullptr); },
      [=]() { failureTask(input, "SimpleProcessManager", input.getInstruction()->getDescription(), nullptr); });

  // Dump the taskflow
  if (debug_)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + "simple_process_manager-" + tesseract_common::getTimestampString() +
                  ".dot");
    taskflow_container_.taskflow->dump(out_data);
    out_data.close();
  }

  return true;
}

bool SimpleProcessManager::execute()
{
  DebugObserver::Ptr debug_observer;
  std::shared_ptr<tf::TFProfObserver> profile_observer;
  if (debug_)
    debug_observer = executor_.make_observer<DebugObserver>("SimpleProcessManager");

  executor_.wait_for_all();
  executor_.run(*(taskflow_container_.taskflow));
  executor_.wait_for_all();

  if (debug_observer != nullptr)
    executor_.remove_observer(debug_observer);

  if (profile_observer != nullptr)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + "simple_process_manager-" + tesseract_common::getTimestampString() +
                  ".json");
    profile_observer->dump(out_data);
    out_data.close();
    executor_.remove_observer(profile_observer);
  }

  clear();  // I believe clear must be called so memory is cleaned up

  return process_input_->isAborted();
}

bool SimpleProcessManager::terminate()
{
  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool SimpleProcessManager::clear()

{
  taskflow_container_.clear();
  return true;
}

void SimpleProcessManager::enableDebug(bool enabled) { debug_ = enabled; }

void SimpleProcessManager::enableProfile(bool enabled) { profile_ = enabled; }
