#ifndef TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Plan Instruction (Const)
 */
const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites = true);

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Plan Instruction (Const)
 */
const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites = true);

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Plan Instruction (Non-Const)
 */
PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction,
                                         bool process_child_composites = true);

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Plan Instruction (Non-Const)
 */
PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction,
                                        bool process_child_composites = true);

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Move Instruction (Const)
 */
const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites = true);

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Move Instruction (Const)
 */
const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites = true);

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The first Move Instruction (Non-Const)
 */
MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction,
                                         bool process_child_composites = true);

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * @param composite_instruction Composite Instruction to search
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The last Move Instruction (Non-Const)
 */
MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction,
                                        bool process_child_composites = true);

/**
 * @brief Get number of Move Instruction in a Composite Instruction
 * @param composite_instruction The Composite Instruction to process
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The number of Move Instructions
 */
long getMoveInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites = true);

/**
 * @brief Get number of Plan Instruction in a Composite Instruction
 * @param composite_instruction The Composite Instruction to process
 * @param process_child_composites Indicate if child Composite Instructions should be searched
 * @return The number of Plan Instructions
 */
long getPlanInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites = true);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
