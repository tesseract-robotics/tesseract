/**
 * @file random_validator.h
 * @brief Validator that returns randomly
 *
 * @author Matthew Powelson
 * @date July 24. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_RANDOM_VALIDATOR_H
#define TESSERACT_PROCESS_MANAGERS_RANDOM_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <console_bridge/console.h>
#include <taskflow/taskflow.hpp>

#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_command_language/null_instruction.h>

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
// TODO: Create more meaningful validators like collision validators for each of our collision check types and format
// check to verify that the seed matches the instructions
inline int randomValidator(const ProcessInput& /*input*/)
{
  double success_frequency = 0.5;

  Eigen::MatrixX2d limits(1, 2);
  limits << 0, 1;
  Eigen::VectorXd rand = tesseract_common::generateRandomNumber(limits);

  int success = (rand[0] > success_frequency) ? 0 : 1;
  std::cout << "randomValidator success: " + std::to_string(success) + "\n";
  return success;
}

}  // namespace tesseract_planning

#endif
