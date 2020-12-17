/**
 * @file raster_taskflow.cpp
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

#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

RasterTaskflow::RasterTaskflow(TaskflowGenerator::UPtr freespace_taskflow_generator,
                               TaskflowGenerator::UPtr transition_taskflow_generator,
                               TaskflowGenerator::UPtr raster_taskflow_generator,
                               std::string name)
  : freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , transition_taskflow_generator_(std::move(transition_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , name_(name)
{
}

const std::string& RasterTaskflow::getName() const { return name_; }

TaskflowContainer RasterTaskflow::generateTaskflow(ProcessInput input,
                                                   std::function<void()> done_cb,
                                                   std::function<void()> error_cb)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    throw std::runtime_error("Invalid Process Input");
  }

  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);
  container.input = container.taskflow->emplace([]() {}).name(name_ + ": Input Task");
  std::vector<tf::Task> tasks;

  // Generate all of the raster tasks. They don't depend on anything
  std::size_t raster_idx = 0;
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
    TaskflowContainer sub_container = raster_taskflow_generator_->generateTaskflow(
        raster_input,
        [=]() { successTask(input, name_, raster_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, raster_input.getInstruction()->getDescription(), error_cb); });

    auto raster_step =
        container.taskflow->composed_of(*(sub_container.taskflow))
            .name("Raster #" + std::to_string(raster_idx + 1) + ": " + raster_input.getInstruction()->getDescription());
    container.containers.push_back(std::move(sub_container));
    container.input.precede(raster_step);
    tasks.push_back(raster_step);
    raster_idx++;
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
    TaskflowContainer sub_container = transition_taskflow_generator_->generateTaskflow(
        transition_input,
        [=]() { successTask(input, name_, transition_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, transition_input.getInstruction()->getDescription(), error_cb); });

    auto transition_step = container.taskflow->composed_of(*(sub_container.taskflow))
                               .name("Transition #" + std::to_string(transition_idx + 1) + ": " +
                                     transition_input.getInstruction()->getDescription());
    container.containers.push_back(std::move(sub_container));

    // Each transition is independent and thus depends only on the adjacent rasters
    transition_step.succeed(tasks[transition_idx]);
    transition_step.succeed(tasks[transition_idx + 1]);

    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  ProcessInput from_start_input = input[0];
  from_start_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()->getStartInstruction());
  from_start_input.setEndInstruction(std::vector<std::size_t>({ 1 }));
  TaskflowContainer sub_container1 = freespace_taskflow_generator_->generateTaskflow(
      from_start_input,
      [=]() { successTask(input, name_, from_start_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, from_start_input.getInstruction()->getDescription(), error_cb); });

  auto from_start = container.taskflow->composed_of(*(sub_container1.taskflow))
                        .name("From Start: " + from_start_input.getInstruction()->getDescription());
  container.containers.push_back(std::move(sub_container1));
  tasks[0].precede(from_start);

  // Plan to_end - preceded by the last raster
  ProcessInput to_end_input = input[input.size() - 1];
  to_end_input.setStartInstruction(std::vector<std::size_t>({ input.size() - 2 }));
  TaskflowContainer sub_container2 = freespace_taskflow_generator_->generateTaskflow(
      to_end_input,
      [=]() { successTask(input, name_, to_end_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, to_end_input.getInstruction()->getDescription(), error_cb); });

  auto to_end = container.taskflow->composed_of(*(sub_container2.taskflow))
                    .name("To End: " + to_end_input.getInstruction()->getDescription());
  container.containers.push_back(std::move(sub_container2));
  tasks.back().precede(to_end);

  return container;
}

bool RasterTaskflow::checkProcessInput(const tesseract_planning::ProcessInput& input) const
{
  // -------------
  // Check Input
  // -------------
  if (!input.env)
  {
    CONSOLE_BRIDGE_logError("ProcessInput env is a nullptr");
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
