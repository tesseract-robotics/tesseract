/**
 * @file filter_functions.h
 * @brief Contains filter functions used for flattening and locating instructions in composites
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
#ifndef TESSERACT_COMMAND_LANGUAGE_UTILS_FILTER_FUNCTIONS_H
#define TESSERACT_COMMAND_LANGUAGE_UTILS_FILTER_FUNCTIONS_H

#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
/**
 * @brief This is used for filtering only what you want in the vector
 *
 * The first parameter is the instruction consider, the second is it's parent composite instruction, and the third
 * indicates if the parent composite is the top most composite
 *
 * The filter should return true when the instruction passed should be included not throw.
 */
using flattenFilterFn =
    std::function<bool(const Instruction&, const CompositeInstruction&, bool parent_is_first_composite)>;
using locateFilterFn =
    std::function<bool(const Instruction&, const CompositeInstruction&, bool parent_is_first_composite)>;

bool moveFilter(const Instruction& instruction, const CompositeInstruction& composite, bool parent_is_first_composite);

bool planFilter(const Instruction& instruction, const CompositeInstruction& composite, bool parent_is_first_composite);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
