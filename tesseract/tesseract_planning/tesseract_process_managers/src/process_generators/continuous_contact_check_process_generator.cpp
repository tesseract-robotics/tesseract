/**
 * @file continuous_contact_check_process_generator.cpp
 * @brief Continuous collision check trajectory
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

#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
ContinuousContactCheckProcessGenerator::ContinuousContactCheckProcessGenerator(std::string name)
  : name_(std::move(name))
{
}

ContinuousContactCheckProcessGenerator::ContinuousContactCheckProcessGenerator(double longest_valid_segment_length,
                                                                               double contact_distance,
                                                                               std::string name)
  : name_(std::move(name))
  , longest_valid_segment_length_(longest_valid_segment_length)
  , contact_distance_(contact_distance)
{
  if (longest_valid_segment_length_ <= 0)
  {
    CONSOLE_BRIDGE_logWarn("ContinuousContactCheckProcessGenerator: Invalid longest valid segment. Defaulting to 0.05");
    longest_valid_segment_length_ = 0.05;
  }
}

const std::string& ContinuousContactCheckProcessGenerator::getName() const { return name_; }

std::function<void()> ContinuousContactCheckProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> ContinuousContactCheckProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int ContinuousContactCheckProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  Instruction* input_results = input.getResults();
  if (!isCompositeInstruction(*input_results))
  {
    CONSOLE_BRIDGE_logError("Input seed to TrajOpt Planner must be a composite instruction");
    return 0;
  }

  // Get state solver
  tesseract_environment::StateSolver::Ptr state_solver = input.tesseract->getEnvironment()->getStateSolver();
  tesseract_collision::ContinuousContactManager::Ptr manager =
      input.tesseract->getEnvironment()->getContinuousContactManager();
  manager->setContactDistanceThreshold(contact_distance_);

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

  const auto* ci = input_results->cast_const<CompositeInstruction>();
  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, *ci, longest_valid_segment_length_))
  {
    CONSOLE_BRIDGE_logInform("Results are not contact free for process input: %s!",
                             input_results->getDescription().c_str());
    for (std::size_t i = 0; i < contacts.size(); i++)
      for (const auto& contact_vec : contacts[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logDebug(("timestep: " + std::to_string(i) + " Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + " Dist: " + std::to_string(contact.distance))
                                      .c_str());
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Continuous contact check succeeded");
  return 1;
}

void ContinuousContactCheckProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool ContinuousContactCheckProcessGenerator::getAbort() const { return abort_; }
void ContinuousContactCheckProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
