/**
 * @file fixed_size_assign_position.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 24, 2020
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
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
CompositeInstruction fixedSizeAssignStateWaypoint(const Eigen::Ref<const Eigen::VectorXd>& position,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  const ManipulatorInfo& manip_info,
                                                  int steps)
{
  assert(!(manip_info.empty() && base_instruction.getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

  // Get Kinematics Object
  auto fwd_kin = request.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(mi.manipulator);

  // Convert to MoveInstructions
  MoveInstructionType mv_type;
  if (base_instruction.isLinear())
    mv_type = MoveInstructionType::LINEAR;
  else if (base_instruction.isFreespace())
    mv_type = MoveInstructionType::FREESPACE;
  else
    throw std::runtime_error("fixedSizeAssignStateWaypoint: Unsupport plan instruction type!");

  CompositeInstruction composite;
  composite.setManipulatorInfo(mi);

  for (int i = 1; i <= steps; ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), position), mv_type);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    move_instruction.setProfile(base_instruction.getProfile());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeAssignStateWaypoint(const JointWaypoint& start,
                                                  const CartesianWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  const ManipulatorInfo& manip_info,
                                                  int steps)
{
  // Joint waypoints should have joint names
  assert(static_cast<long>(start.joint_names.size()) == start.size());

  return fixedSizeAssignStateWaypoint(start, base_instruction, request, manip_info, steps);
}

CompositeInstruction fixedSizeAssignStateWaypoint(const CartesianWaypoint& /*start*/,
                                                  const JointWaypoint& end,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  const ManipulatorInfo& manip_info,
                                                  int steps)
{
  // Joint waypoints should have joint names
  assert(static_cast<long>(end.joint_names.size()) == end.size());

  return fixedSizeAssignStateWaypoint(end, base_instruction, request, manip_info, steps);
}

CompositeInstruction fixedSizeAssignStateWaypoint(const CartesianWaypoint& /*start*/,
                                                  const CartesianWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  const ManipulatorInfo& manip_info,
                                                  int steps)
{
  assert(!(manip_info.empty() && base_instruction.getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());
  auto fwd_kin = request.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(mi.manipulator);
  Eigen::VectorXd position = request.env_state->getJointValues(fwd_kin->getJointNames());
  return fixedSizeAssignStateWaypoint(position, base_instruction, request, manip_info, steps);
}

}  // namespace tesseract_planning
