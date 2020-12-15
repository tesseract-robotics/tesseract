/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
void generateNaiveSeedHelper(CompositeInstruction& composite_instructions,
                             const tesseract_environment::Environment& env,
                             const tesseract_environment::EnvState& env_state,
                             const ManipulatorInfo& manip_info)
{
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateNaiveSeedHelper(*(i.cast<CompositeInstruction>()), env, env_state, manip_info);
    }
    else if (isPlanInstruction(i))
    {
      PlanInstruction* base_instruction = i.cast<PlanInstruction>();
      ManipulatorInfo mi = manip_info.getCombined(base_instruction->getManipulatorInfo());

      CompositeInstruction ci;
      ci.setProfile(base_instruction->getProfile());
      ci.setDescription(base_instruction->getDescription());
      ci.setManipulatorInfo(base_instruction->getManipulatorInfo());

      auto fwd_kin = env.getManipulatorManager()->getFwdKinematicSolver(mi.manipulator);
      Eigen::VectorXd jv = env_state.getJointValues(fwd_kin->getJointNames());

      // Get move type base on base instruction type
      MoveInstructionType move_type;
      if (base_instruction->isLinear())
        move_type = MoveInstructionType::LINEAR;
      else if (base_instruction->isFreespace())
        move_type = MoveInstructionType::FREESPACE;
      else
        throw std::runtime_error("generateNaiveSeed: Unsupported Move Instruction Type!");

      if (isStateWaypoint(base_instruction->getWaypoint()))
      {
        MoveInstruction move_instruction(base_instruction->getWaypoint(), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }
      else if (isJointWaypoint(base_instruction->getWaypoint()))
      {
        const auto* jwp = base_instruction->getWaypoint().cast<JointWaypoint>();
        MoveInstruction move_instruction(StateWaypoint(jwp->joint_names, jwp->waypoint), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }
      else
      {
        MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), jv), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }

      i = ci;
    }
  }
}

CompositeInstruction generateNaiveSeed(const CompositeInstruction& composite_instructions,
                                       const tesseract_environment::Environment& env)
{
  if (!composite_instructions.hasStartInstruction())
    throw std::runtime_error("Top most composite instruction is missing start instruction!");

  tesseract_environment::EnvState::ConstPtr env_state = env.getCurrentState();
  CompositeInstruction seed = composite_instructions;
  ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

  Waypoint wp = NullWaypoint();
  ManipulatorInfo base_mi;
  std::string description;
  std::string profile;
  if (isPlanInstruction(seed.getStartInstruction()))
  {
    const auto* pi = seed.getStartInstruction().cast<PlanInstruction>();
    wp = pi->getWaypoint();
    base_mi = pi->getManipulatorInfo();
    description = pi->getDescription();
    profile = pi->getProfile();
  }
  else if (isMoveInstruction(seed.getStartInstruction()))
  {
    const auto* pi = seed.getStartInstruction().cast<MoveInstruction>();
    wp = pi->getWaypoint();
    base_mi = pi->getManipulatorInfo();
    description = pi->getDescription();
    profile = pi->getProfile();
  }
  else
    throw std::runtime_error("Top most composite instruction start instruction has invalid waypoint type!");

  ManipulatorInfo start_mi = mi.getCombined(base_mi);
  auto fwd_kin = env.getManipulatorManager()->getFwdKinematicSolver(start_mi.manipulator);
  Eigen::VectorXd jv = env_state->getJointValues(fwd_kin->getJointNames());

  if (isStateWaypoint(wp))
  {
    MoveInstruction move_instruction(wp, MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }
  else if (isJointWaypoint(wp))
  {
    const auto* jwp = wp.cast<JointWaypoint>();
    MoveInstruction move_instruction(StateWaypoint(jwp->joint_names, jwp->waypoint), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }
  else
  {
    MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), jv), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }

  generateNaiveSeedHelper(seed, env, *env_state, mi);
  return seed;
}
};  // namespace tesseract_planning
