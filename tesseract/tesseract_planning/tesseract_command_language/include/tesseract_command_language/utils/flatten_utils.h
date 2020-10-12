/**
 * @file flatten_utils.h
 * @brief Utilities for flattening composites
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_UTILS_FLATTEN_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_UTILS_FLATTEN_UTILS_H

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>

namespace tesseract_planning
{
/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction
 * @param composite_instruction Input composite instruction to be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flatten(CompositeInstruction& composite_instruction,
                                                         flattenFilterFn filter = nullptr);

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 * @param instruction Input composite instruction to be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>> flatten(const CompositeInstruction& composite_instruction,
                                                               flattenFilterFn filter = nullptr);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flattenToPattern(CompositeInstruction& composite_instruction,
                                                                  const CompositeInstruction& pattern,
                                                                  flattenFilterFn filter = nullptr);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>>
flattenToPattern(const CompositeInstruction& composite_instruction,
                 const CompositeInstruction& pattern,
                 flattenFilterFn filter = nullptr);
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
