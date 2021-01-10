/**
 * @file get_instruction_utils.h
 * @brief Utilities such as get<First/Last><Type>Instruction
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
#ifndef TESSERACT_COMMAND_LANGUAGE_UTILS_GET_INSTRUCTION_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_UTILS_GET_INSTRUCTION_UTILS_H

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils/filter_functions.h>

namespace tesseract_planning
{
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
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Move Instruction (Non-Const)
 */
inline MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getFirstInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Move Instruction (Const)
 */
inline const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getFirstInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast_const<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Plan Instruction (Non-Const)
 */
inline PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getFirstInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Plan Instruction (Const)
 */
inline const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getFirstInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast_const<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Move Instruction (Non-Const)
 */
inline MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getLastInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Move Instruction (Const)
 */
inline const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getLastInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast_const<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Plan Instruction (Non-Const)
 */
inline PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getLastInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Plan Instruction (Const)
 */
inline const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getLastInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast_const<PlanInstruction>();

  return nullptr;
}

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
 * @brief Get number of Move Instruction in a Composite Instruction
 * This does not consider the start instruction in the child composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @return The number of Move Instructions
 */
inline long getMoveInstructionCount(const CompositeInstruction& composite_instruction)
{
  return getInstructionCount(composite_instruction, moveFilter);
}

/**
 * @brief Get number of Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in the child composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @return The number of Plan Instructions
 */
inline long getPlanInstructionCount(const CompositeInstruction& composite_instruction)
{
  return getInstructionCount(composite_instruction, planFilter);
}
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
