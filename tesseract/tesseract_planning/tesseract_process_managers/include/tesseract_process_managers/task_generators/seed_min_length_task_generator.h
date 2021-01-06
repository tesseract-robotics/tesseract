/**
 * @file seed_length_task_generator.h
 * @brief Process generator for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, accelleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_SEED_MIN_LENGTH_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_SEED_MIN_LENGTH_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
class SeedMinLengthTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<SeedMinLengthTaskGenerator>;

  SeedMinLengthTaskGenerator(std::string name = "Seed Min Length");

  SeedMinLengthTaskGenerator(long min_length, std::string name = "Seed Min Length");

  ~SeedMinLengthTaskGenerator() override = default;
  SeedMinLengthTaskGenerator(const SeedMinLengthTaskGenerator&) = delete;
  SeedMinLengthTaskGenerator& operator=(const SeedMinLengthTaskGenerator&) = delete;
  SeedMinLengthTaskGenerator(SeedMinLengthTaskGenerator&&) = delete;
  SeedMinLengthTaskGenerator& operator=(SeedMinLengthTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override;

  void process(TaskInput input, std::size_t unique_id) const override;

private:
  long min_length_{ 10 };

  void subdivide(CompositeInstruction& composite,
                 const CompositeInstruction& current_composite,
                 Instruction& start_instruction,
                 int subdivisions) const;
};

class SeedMinLengthTaskInfo : public TaskInfo
{
public:
  SeedMinLengthTaskInfo(std::size_t unique_id, std::string name = "Seed Min Length");
};
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_SEED_MIN_LENGTH_TASK_GENERATOR_H
