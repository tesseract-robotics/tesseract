/**
 * @file command_language_utils.h
 * @brief
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
#ifndef TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
/**
 * @brief This is used for filtering only what you want in the vector
 *
 * The first parameter is the instruction consider, the second is it's parent composite instruction, and the third is
 * indicates if the parent composite is the top most composite
 * For example an Instruction that is part of a composite
 */
using flattenFilterFn =
    std::function<bool(const Instruction&, const CompositeInstruction&, bool parent_is_first_composite)>;
using locateFilterFn =
    std::function<bool(const Instruction&, const CompositeInstruction&, bool parent_is_first_composite)>;

/**
 * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
 * @param composite_instruction Composite Instruction to search
 * @param locate_filter The filter to indicate if an instruction should be considered
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Instruction (Const)
 */
const Instruction* getFirstInstruction(const CompositeInstruction& composite_instruction,
                                       locateFilterFn locate_filter = nullptr,
                                       bool process_child_composites = true);

/**
 * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
 * @param composite_instruction Composite Instruction to search
 * @param locate_filter The filter to indicate if an instruction should be considered
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Instruction (Non-Const)
 */
Instruction* getFirstInstruction(CompositeInstruction& composite_instruction,
                                 locateFilterFn locate_filter = nullptr,
                                 bool process_child_composites = true);

/**
 * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
 * @param composite_instruction Composite Instruction to search
 * @param locate_filter The filter to indicate if an instruction should be considered
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The Last Instruction (Const)
 */
const Instruction* getLastInstruction(const CompositeInstruction& composite_instruction,
                                      locateFilterFn locate_filter = nullptr,
                                      bool process_child_composites = true);

/**
 * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
 * @param composite_instruction Composite Instruction to search
 * @param locate_filter The filter to indicate if an instruction should be considered
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The Last Instruction (Non-Const)
 */
Instruction* getLastInstruction(CompositeInstruction& composite_instruction,
                                locateFilterFn locate_filter = nullptr,
                                bool process_child_composites = true);

/**
 * @brief Get number of Instruction in a Composite Instruction
 * @param composite_instruction The Composite Instruction to process
 * @param locate_filter The filter to indicate if an instruction should be considered
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The number of Instructions
 */
long getInstructionCount(const CompositeInstruction& composite_instruction,
                         locateFilterFn locate_filter = nullptr,
                         bool process_child_composites = true);

/**
 * @brief Extracts joint position from waypoint types that store joint positions
 * If invalid waypoint type then exception is thrown
 * @param waypoint The waypoint to extract joint position
 * @return The joint position
 */
const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint);

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

/**
 * @brief This creates a seed by looping over and replacing every plan instruction with a composite instruction
 * @param instructions
 * @return
 */
CompositeInstruction generateSkeletonSeed(const CompositeInstruction& composite_instructions);

/**
 * @brief This loops over the instructions validates the structure
 *
 * Every plan instruction in /p composite_instruction should have a cooresponding CompositeInstruction
 *
 * @param composite_instructions
 * @param composite_seed
 * @return
 */
bool validateSeedStructure(const CompositeInstruction& composite_instructions,
                           const CompositeInstruction& composite_seed);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
