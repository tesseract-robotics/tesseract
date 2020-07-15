/**
 * @file random_process_generator.h
 * @brief Generates a process that returns randomly
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
#ifndef TESSERACT_PROCESS_MANAGERS_RANDOM_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_RANDOM_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <console_bridge/console.h>
#include <taskflow/taskflow.hpp>

#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
/**
 * @brief Generates instances of processes that return random values.
 */
class RandomProcessGenerator : public ProcessGenerator
{
public:
  using Ptr = std::shared_ptr<RandomProcessGenerator>;
  using ConstPtr = std::shared_ptr<const RandomProcessGenerator>;

  std::function<void()> generateTask(ProcessInput input) override
  {
    task_inputs_.push_back(input);

    return std::bind(&RandomProcessGenerator::process, this, task_inputs_.back());
  }

  std::function<int()> generateConditionalTask(ProcessInput input) override
  {
    task_inputs_.push_back(input);

    return std::bind(&RandomProcessGenerator::conditionalProcess, this, task_inputs_.back());
  }

  void process(const ProcessInput& /*results*/) const { std::cout << name + "\n"; }

  int conditionalProcess(const ProcessInput& /*results*/) const
  {
    Eigen::MatrixX2d limits(1, 2);
    limits << 0, 1;
    Eigen::VectorXd rand = tesseract_common::generateRandomNumber(limits);

    int success = (rand[0] > success_frequency) ? 0 : 1;
    std::cout << name + "  Success: " + std::to_string(success) + "\n";
    return success;
  }

  /** @brief Between 0 and 1. Likelyhood this process with return true (if using conditionalProcess) */
  double success_frequency{ 0.5 };

private:
  std::vector<ProcessInput> task_inputs_;
};

}  // namespace tesseract_planning

#endif
