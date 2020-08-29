/**
 * @file process_input.h
 * @brief Process input
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_INPUT_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_INPUT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/manipulator_info.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract/tesseract.h>

namespace tesseract_planning
{
/**
 * @brief This struct is passed as an input to each process in the decision tree
 *
 * Note that it does not have ownership of any of its members (except the pointer). This means that if a ProcessInput
 * spawns a child that is a subset, it does not have to remain in scope as the references will still be valid
 */
struct ProcessInput
{
  using Ptr = std::shared_ptr<ProcessInput>;
  using ConstPtr = std::shared_ptr<const ProcessInput>;

  ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
               const Instruction* instruction,
               const ManipulatorInfo& manip_info,
               Instruction* seed);

  ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
               const Instruction* instruction,
               const ManipulatorInfo& manip_info,
               const PlannerProfileRemapping& plan_profile_remapping,
               const PlannerProfileRemapping& composite_profile_remapping,
               Instruction* seed);

  ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
               const Instruction* instruction,
               const PlannerProfileRemapping& plan_profile_remapping,
               const PlannerProfileRemapping& composite_profile_remapping,
               Instruction* seed);

  ProcessInput(tesseract::Tesseract::ConstPtr tesseract, const Instruction* instruction, Instruction* seed);

  /**
   * @brief Creates a sub-ProcessInput from instruction[index] and seed[index]
   * @param index sub-Instruction used to create the ProcessInput
   * @return A ProcessInput containing a subset of the original's instructions
   */
  ProcessInput operator[](std::size_t index);

  /**
   * @brief Gets the number of instructions contained in the ProcessInput
   * @return 1 instruction if not a composite, otherwise size of the composite @todo Should this be -1, becuase
   * composite size could be 1, 0, or other?
   */
  std::size_t size();

  /** @brief Tesseract associated with current state of the system */
  const tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Instructions to be carried out by process */
  const Instruction* instruction;

  /** @brief Global Manipulator Information */
  const ManipulatorInfo& manip_info;

  /**
   * @brief This allows the remapping of the Plan Profile identified in the command language to a specific profile for a
   * given motion planner.
   */
  const PlannerProfileRemapping& plan_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner.
   */
  const PlannerProfileRemapping& composite_profile_remapping;

  /** @brief Results/Seed for this process */
  Instruction* results;

  // These are used to store alternative start and end instructions
  Instruction start_instruction;
  Instruction end_instruction;
  const Instruction* start_instruction_ptr{ nullptr };
  const Instruction* end_instruction_ptr{ nullptr };
};

}  // namespace tesseract_planning

#endif
