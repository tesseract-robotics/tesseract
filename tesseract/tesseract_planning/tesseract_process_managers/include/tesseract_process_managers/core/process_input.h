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
#include <atomic>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_interface.h>
#include <tesseract_process_managers/core/process_info.h>

#include <tesseract_motion_planners/core/profile_dictionary.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/types.h>

#include <tesseract_environment/core/environment.h>

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

  ProcessInput(tesseract_environment::Environment::ConstPtr env,
               const Instruction* instruction,
               const ManipulatorInfo& manip_info,
               Instruction* seed,
               bool has_seed,
               ProfileDictionary::ConstPtr profiles);

  ProcessInput(tesseract_environment::Environment::ConstPtr env,
               const Instruction* instruction,
               const ManipulatorInfo& manip_info,
               const PlannerProfileRemapping& plan_profile_remapping,
               const PlannerProfileRemapping& composite_profile_remapping,
               Instruction* seed,
               bool has_seed,
               ProfileDictionary::ConstPtr profiles);

  ProcessInput(tesseract_environment::Environment::ConstPtr env,
               const Instruction* instruction,
               const PlannerProfileRemapping& plan_profile_remapping,
               const PlannerProfileRemapping& composite_profile_remapping,
               Instruction* seed,
               bool has_seed,
               ProfileDictionary::ConstPtr profiles);

  ProcessInput(tesseract_environment::Environment::ConstPtr env,
               const Instruction* instruction,
               Instruction* seed,
               bool has_seed,
               ProfileDictionary::ConstPtr profiles);

  /** @brief Tesseract associated with current state of the system */
  const tesseract_environment::Environment::ConstPtr env;

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

  /** @brief The Profiles to use */
  const ProfileDictionary::ConstPtr profiles;

  /**
   * @brief This indicates if a seed was provided
   * @details In the case of the raster process planner a skeleton seed is provided which make it
   * computationaly intensive to determine if a seed was provide so this was added.
   */
  const bool has_seed{ false };

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

  /**
   * @brief Get the process inputs instructions
   * @return A const pointer to the instruction
   */
  const Instruction* getInstruction() const;

  /**
   * @brief Get the process inputs results instruction
   * @return A pointer to the results instruction
   */
  Instruction* getResults();

  /** @brief The process interface for checking success and aborting active process */
  ProcessInterface::Ptr getProcessInterface();

  /**
   * @brief Check if process has been aborted
   * @details This accesses the internal process interface class
   * @return True if aborted otherwise false;
   */
  bool isAborted() const;

  /**
   * @brief Abort the process intput
   * @details This accesses the internal process interface class to abort the process
   */
  void abort();

  void setStartInstruction(Instruction start);
  void setStartInstruction(std::vector<std::size_t> start);
  Instruction getStartInstruction() const;

  void setEndInstruction(Instruction end);
  void setEndInstruction(std::vector<std::size_t> end);
  Instruction getEndInstruction() const;

  void addProcessInfo(const ProcessInfo::ConstPtr& process_info);
  ProcessInfo::ConstPtr getProcessInfo(const std::size_t& index) const;
  std::map<std::size_t, ProcessInfo::ConstPtr> getProcessInfoMap() const;

protected:
  /** @brief Instructions to be carried out by process */
  const Instruction* instruction_;

  /** @brief Results/Seed for this process */
  Instruction* results_;

  /** @brief The indicies used to access this process inputs instructions and results */
  std::vector<std::size_t> instruction_indice_;

  /** @brief This proccess inputs start instruction */
  Instruction start_instruction_{ NullInstruction() };

  /** @brief Indices to the start instruction in the results data struction */
  std::vector<std::size_t> start_instruction_indice_;

  /** @brief This proccess inputs end instruction */
  Instruction end_instruction_{ NullInstruction() };

  /** @brief Indices to the end instruction in the results data struction */
  std::vector<std::size_t> end_instruction_indice_;

  std::shared_ptr<ProcessInfoContainer> process_infos_{ std::make_shared<ProcessInfoContainer>() };

  /** @brief Used to store if process input is aborted which is thread safe */
  ProcessInterface::Ptr interface_{ std::make_shared<ProcessInterface>() };
};

}  // namespace tesseract_planning

#endif
