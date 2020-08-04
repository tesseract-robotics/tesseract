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
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Plan Instruction (Const)
 */
const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites = true);

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Plan Instruction (Const)
 */
const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites = true);

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched.
 * @return The first Plan Instruction (Non-Const)
 */
PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction,
                                         bool process_child_composites = true);

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Plan Instruction (Non-Const)
 */
PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction,
                                        bool process_child_composites = true);

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Move Instruction (Const)
 */
const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites = true);

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Move Instruction (Const)
 */
const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites = true);

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Move Instruction (Non-Const)
 */
MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction,
                                         bool process_child_composites = true);

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Move Instruction (Non-Const)
 */
MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction,
                                        bool process_child_composites = true);

/**
 * @brief Get number of Move Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The number of Move Instructions
 */
long getMoveInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites = true);

/**
 * @brief Get number of Plan Instruction in a Composite Instruction
 * This does consider the start instruction in the composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The number of Plan Instructions
 */
long getPlanInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites = true);

/**
 * @brief Extracts joint position from waypoint types that store joint positions
 * If invalid waypoint type then exception is thrown
 * @param waypoint The waypoint to extract joint position
 * @return The joint position
 */
const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint);

/** @brief This is used for filtering only what you want in the vector **/
using flattenFilter = std::function<bool(const Instruction&)>;

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param composite_instruction Input composite instruction to be flattened
 * @param filter Used to filter only what. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flatten(CompositeInstruction& composite_instruction,
                                                         const flattenFilter& filter = nullptr);

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param instruction Input composite instruction to be flattened
 * @param filter Used to filter only what. Should return true to include otherwise false
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>> flatten(const CompositeInstruction& composite_instruction,
                                                               const flattenFilter& filter = nullptr);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param include_composite Default: false. If true, CompositeInstructions will be included in the final flattened
 * vector
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flattenToPattern(CompositeInstruction& composite_instruction,
                                                                  const CompositeInstruction& pattern,
                                                                  bool process_child_composites = false);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param include_composite Default: false. If true, CompositeInstructions will be included in the final flattened
 * vector
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>>
flattenToPattern(const CompositeInstruction& composite_instruction,
                 const CompositeInstruction& pattern,
                 bool process_child_composites = false);

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
