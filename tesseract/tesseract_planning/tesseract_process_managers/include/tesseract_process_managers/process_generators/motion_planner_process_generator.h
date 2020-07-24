/**
 * @file motion_planner_process_generator.h
 * @brief Generates a motion planning process
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
#ifndef TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_command_language/null_instruction.h>

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
class MotionPlannerProcessGenerator : public ProcessGenerator
{
public:
  using Ptr = std::shared_ptr<MotionPlannerProcessGenerator>;
  using ConstPtr = std::shared_ptr<const MotionPlannerProcessGenerator>;

  MotionPlannerProcessGenerator(std::shared_ptr<MotionPlanner>);

  std::function<void()> generateTask(ProcessInput input) override;

  std::function<void()> generateTask(ProcessInput input, const Instruction& start_instruction) override;

  std::function<void()> generateTask(ProcessInput input,
                                     const Instruction& start_instruction,
                                     const Instruction& end_instruction) override;

  std::function<int()> generateConditionalTask(ProcessInput input) override;

  std::function<int()> generateConditionalTask(ProcessInput input, const Instruction& start_instruction) override;

  std::function<int()> generateConditionalTask(ProcessInput input,
                                               const Instruction& start_instruction,
                                               const Instruction& end_instruction) override;

  int conditionalProcess(const ProcessInput& input,
                         const Instruction& start_instruction,
                         const Instruction& end_instruction) const;

  void process(const ProcessInput& input,
               const Instruction& start_instruction,
               const Instruction& end_instruction) const;

  std::shared_ptr<MotionPlanner> planner;

  bool getAbort() const override;
  void setAbort(bool abort) override;

  /** @brief TODO: Figure out exactly how to do this. I have gone back and forth, but I think it might be easiest to not
   * make these tasks in the taskflow and just it this internal to conditionalProcess. */
  std::vector<std::function<int(const ProcessInput&)>> validators;

private:
  /** @brief We have to keep these alive for taskflow*/
  std::vector<ProcessInput> task_inputs_;

  NullInstruction null_instruction_;

  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };
};

}  // namespace tesseract_planning

#endif
