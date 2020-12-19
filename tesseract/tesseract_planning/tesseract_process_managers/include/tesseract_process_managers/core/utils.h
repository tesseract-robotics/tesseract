/**
 * @file utils.h
 * @brief Tesseract process managers utility functions
 *
 * @author Matthew Powelson
 * @author Levi Armstrong
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
#ifndef TESSERACT_PROCESS_MANAGERS_UTILS_H
#define TESSERACT_PROCESS_MANAGERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/types.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_process_managers/core/process_input.h>

namespace tesseract_planning
{
/**
 * @brief The default success task to be used
 * @param name The name
 * @param message A detailed message
 * @param user_callback A user callback function
 */
void successTask(const ProcessInput& /*instruction*/,
                 const std::string& name,
                 const std::string& message,
                 const TaskflowVoidFn& user_callback = nullptr);

/**
 * @brief The default failure task to be used
 * @details This will call the abort function of the ProcessInput provided
 * @param name The name
 * @param message A detailed message
 * @param user_callback A user callback function
 */
void failureTask(ProcessInput instruction,
                 const std::string& name,
                 const std::string& message,
                 const TaskflowVoidFn& user_callback = nullptr);

/**
 * @brief Check if composite is empty along with children composites
 * @param composite The composite to check
 * @return True if empty otherwise false
 */
bool isCompositeEmpty(const CompositeInstruction& composite);

/**
 * @brief Check if the input has a seed
 * @details It checks if a any composite instruction is empty in the results data structure
 * @param input The process input
 * @return One if seed exists, otherwise zero
 */
int hasSeedTask(ProcessInput input);

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_UTILS_H
