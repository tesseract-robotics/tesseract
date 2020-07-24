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

namespace tesseract_planning
{
CompositeInstruction fixedSizeAssignJointPosition(const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  int steps)
{
  CompositeInstruction composite;

  // Get current state
  std::string manipulator = base_instruction.getManipulatorInfo().manipulator;
  auto fk = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (!fk)
    throw std::runtime_error("fixedSizeAssignJointPosition: failed to find forward kinematics solution!");
  JointWaypoint wp = request.env_state->getJointValues(fk->getJointNames());

  // Convert to MoveInstructions
  for (int i = 1; i < steps; ++i)
  {
    tesseract_planning::MoveInstruction move_instruction(
        wp, MoveInstructionType::FREESPACE);  // TODO: Set this type based on base_instruction (freespace or linear)
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

CompositeInstruction fixedSizeAssignJointPosition(const JointWaypoint& /*start*/,
                                                  const JointWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  int steps)
{
  return fixedSizeAssignJointPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignJointPosition(const JointWaypoint& /*start*/,
                                                  const CartesianWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  int steps)
{
  return fixedSizeAssignJointPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignJointPosition(const CartesianWaypoint& /*start*/,
                                                  const JointWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  int steps)
{
  return fixedSizeAssignJointPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignJointPosition(const CartesianWaypoint& /*start*/,
                                                  const CartesianWaypoint& /*end*/,
                                                  const PlanInstruction& base_instruction,
                                                  const PlannerRequest& request,
                                                  int steps)
{
  return fixedSizeAssignJointPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignCartesianPosition(const PlanInstruction& base_instruction,
                                                      const PlannerRequest& request,
                                                      int steps)
{
  CompositeInstruction composite;

  // Initialize
  auto fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(
      base_instruction.getManipulatorInfo().manipulator);
  auto world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());
  Eigen::Isometry3d tcp = base_instruction.getManipulatorInfo().tcp;

  // Get current state
  std::string manipulator = base_instruction.getManipulatorInfo().manipulator;
  auto fk = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  Eigen::VectorXd joint_state = request.env_state->getJointValues(fk->getJointNames());

  // Calculate FK for the current state
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p1, joint_state))
    throw std::runtime_error("fixedSizeAssignCartesianPosition: failed to find forward kinematics solution!");
  p1 = world_to_base * p1 * tcp;
  CartesianWaypoint wp = p1;

  // Convert to MoveInstructions
  for (int p = 1; p < steps; ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(wp, MoveInstructionType::LINEAR);  // TODO: Get from
                                                                                            // base_instruction
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeAssignCartesianPosition(const JointWaypoint& /*start*/,
                                                      const JointWaypoint& /*end*/,
                                                      const PlanInstruction& base_instruction,
                                                      const PlannerRequest& request,
                                                      int steps)
{
  return fixedSizeAssignCartesianPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignCartesianPosition(const JointWaypoint& /*start*/,
                                                      const CartesianWaypoint& /*end*/,
                                                      const PlanInstruction& base_instruction,
                                                      const PlannerRequest& request,
                                                      int steps)
{
  return fixedSizeAssignCartesianPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignCartesianPosition(const CartesianWaypoint& /*start*/,
                                                      const JointWaypoint& /*end*/,
                                                      const PlanInstruction& base_instruction,
                                                      const PlannerRequest& request,
                                                      int steps)
{
  return fixedSizeAssignCartesianPosition(base_instruction, request, steps);
}

CompositeInstruction fixedSizeAssignCartesianPosition(const CartesianWaypoint& /*start*/,
                                                      const CartesianWaypoint& /*end*/,
                                                      const PlanInstruction& base_instruction,
                                                      const PlannerRequest& request,
                                                      int steps)
{
  return fixedSizeAssignCartesianPosition(base_instruction, request, steps);
}

}  // namespace tesseract_planning
