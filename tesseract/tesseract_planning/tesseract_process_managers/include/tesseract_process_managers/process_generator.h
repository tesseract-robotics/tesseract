/**
 * @file process_generator.h
 * @brief Process generator
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_input.h>

namespace tesseract_planning
{
/**
 * @brief This is a base class for generating instances of processes as tasks such that they may be executed in
 * parallel. A typical workflow would be taskflow.emplace(process_generator.generateTask(input) override)
 */
class ProcessGenerator
{
public:
  using Ptr = std::shared_ptr<ProcessGenerator>;
  using ConstPtr = std::shared_ptr<const ProcessGenerator>;

  virtual std::function<void()> generateTask(ProcessInput input) = 0;

  virtual std::function<void()> generateTask(ProcessInput input, const Instruction& start_instruction) = 0;

  virtual std::function<void()> generateTask(ProcessInput input,
                                             const Instruction& start_instruction,
                                             const Instruction& end_instruction) = 0;

  virtual std::function<int()> generateConditionalTask(ProcessInput input) = 0;

  virtual std::function<int()> generateConditionalTask(ProcessInput input, const Instruction& start_instruction) = 0;

  virtual std::function<int()> generateConditionalTask(ProcessInput input,
                                                       const Instruction& start_instruction,
                                                       const Instruction& end_instruction) = 0;

  std::string name;
};

}  // namespace tesseract_planning

#endif
