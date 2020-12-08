/**
 * @file discrete_contact_check_process_generator.cpp
 * @brief Discrete collision check trajectory
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/discrete_contact_check_process_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
DiscreteContactCheckProcessGenerator::DiscreteContactCheckProcessGenerator(std::string name) : name_(std::move(name))
{
  config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  config.longest_valid_segment_length = 0.05;
  config.collision_margin_data = tesseract_collision::CollisionMarginData(0);
}

DiscreteContactCheckProcessGenerator::DiscreteContactCheckProcessGenerator(double longest_valid_segment_length,
                                                                           double contact_distance,
                                                                           std::string name)
  : name_(std::move(name))
{
  config.longest_valid_segment_length = longest_valid_segment_length;
  config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  config.collision_margin_data = tesseract_collision::CollisionMarginData(contact_distance);
  if (config.longest_valid_segment_length <= 0)
  {
    CONSOLE_BRIDGE_logWarn("DiscreteContactCheckProcessGenerator: Invalid longest valid segment. Defaulting to 0.05");
    config.longest_valid_segment_length = 0.05;
  }
}

const std::string& DiscreteContactCheckProcessGenerator::getName() const { return name_; }

std::function<void()> DiscreteContactCheckProcessGenerator::generateTask(ProcessInput input, std::size_t unique_id)
{
  return [=]() { process(input, unique_id); };
}

std::function<int()> DiscreteContactCheckProcessGenerator::generateConditionalTask(ProcessInput input,
                                                                                   std::size_t unique_id)
{
  return [=]() { return conditionalProcess(input, unique_id); };
}

int DiscreteContactCheckProcessGenerator::conditionalProcess(ProcessInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<DiscreteContactCheckProcessInfo>(unique_id, name_);
  info->return_value = 0;
  input.addProcessInfo(info);

  // --------------------
  // Check that inputs are valid
  // --------------------
  Instruction* input_result = input.getResults();
  if (!isCompositeInstruction(*input_result))
  {
    info->message = "Input seed to DiscreteContactCheckProcessGenerator must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return 0;
  }

  // Get state solver
  tesseract_environment::StateSolver::Ptr state_solver = input.tesseract->getEnvironment()->getStateSolver();
  tesseract_collision::DiscreteContactManager::Ptr manager =
      input.tesseract->getEnvironment()->getDiscreteContactManager();
  manager->setCollisionMarginData(config.collision_margin_data);

  // Set the active links based on the manipulator
  std::vector<std::string> active_links_manip;
  {
    tesseract_environment::AdjacencyMap::Ptr adjacency_map_manip =
        std::make_shared<tesseract_environment::AdjacencyMap>(
            input.tesseract->getEnvironment()->getSceneGraph(),
            input.tesseract->getEnvironment()
                ->getManipulatorManager()
                ->getFwdKinematicSolver(input.manip_info.manipulator)
                ->getActiveLinkNames(),
            input.tesseract->getEnvironment()->getCurrentState()->link_transforms);
    active_links_manip = adjacency_map_manip->getActiveLinkNames();
  }
  manager->setActiveCollisionObjects(active_links_manip);

  const auto* ci = input_result->cast_const<CompositeInstruction>();
  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, *ci, config))
  {
    CONSOLE_BRIDGE_logInform("Results are not contact free for process intput: %s !",
                             input_result->getDescription().c_str());
    for (std::size_t i = 0; i < contacts.size(); i++)
      for (const auto& contact_vec : contacts[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logDebug(("timestep: " + std::to_string(i) + " Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + " Dist: " + std::to_string(contact.distance))
                                      .c_str());
    info->contact_results = contacts;
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Discrete contact check succeeded");
  info->return_value = 1;
  return 1;
}

void DiscreteContactCheckProcessGenerator::process(ProcessInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

DiscreteContactCheckProcessInfo::DiscreteContactCheckProcessInfo(std::size_t unique_id, std::string name)
  : ProcessInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning
