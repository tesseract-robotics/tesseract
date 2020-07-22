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
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SeedGenerator::SeedGenerator(tesseract_environment::EnvState::ConstPtr current_state,
                             tesseract_kinematics::ForwardKinematics::Ptr fwd_kin,
                             tesseract_kinematics::InverseKinematics::Ptr inv_kin,
                             int freespace_segments,
                             int cartesian_segments)
  : current_state(current_state)
  , fwd_kin(fwd_kin)
  , inv_kin(inv_kin)
  , freespace_segments(freespace_segments)
  , cartesian_segments(cartesian_segments)
{
  current_jv = current_state->getJointValues(fwd_kin->getJointNames());
  world_to_base = current_state->link_transforms.at(fwd_kin->getBaseLinkName());
}

CompositeInstruction SeedGenerator::processCompositeInstruction(const CompositeInstruction& instructions)
{
  CompositeInstruction seed;
  for (const auto& instruction : instructions)
  {
    if (isCompositeInstruction(instruction))
    {
      seed.push_back(processCompositeInstruction(*instruction.cast_const<CompositeInstruction>()));
    }
    else if (isPlanInstruction(instruction))
    {
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        CompositeInstruction composite;

        bool is_cwp1 = isCartesianWaypoint(start_waypoint);
        bool is_jwp1 = isJointWaypoint(start_waypoint);
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint());

        assert(is_cwp1 || is_jwp1);
        assert(is_cwp2 || is_jwp2);

        if (is_cwp1 && is_cwp2)
        {
          // If both are cartesian it will cartesian interpolate and use the current state as the seed.
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, *cur_cwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(current_jv);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_jwp2)
        {
          // If one is cartesian and the other is a joint waypoint it will calculate the forward kinematics
          // then cartesian interpolate and set the seed as the provided joint_waypoint.
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p2 = world_to_base * p2 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, p2, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(*cur_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp2 && is_jwp1)
        {
          // If one is cartesian and the other is a joint waypoint it will calculate the forward kinematics
          // then cartesian interpolate and set the seed as the provided joint_waypoint.
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p1 = world_to_base * p1 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(p1, *cur_cwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(*pre_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_jwp1 && is_jwp2)
        {
          // If both are joint waypoints it will calculate the forward kinematics for both then cartesian interpolate
          // and set the seed using joint interpolation between the two.
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p1 = world_to_base * p1 * plan_instruction->getTCP();

          Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p2 = world_to_base * p2 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, cartesian_segments);
          Eigen::MatrixXd joint_poses = interpolate(*pre_jwp, *cur_jwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(joint_poses.col(static_cast<long>(p)));
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          throw std::runtime_error("tesseract_planning::generateSeed: unsupported waypoints provided!");
        }

        seed.push_back(composite);
      }
      else if (plan_instruction->isFreespace())
      {
        CompositeInstruction composite;

        bool is_cwp1 = isCartesianWaypoint(start_waypoint);
        bool is_jwp1 = isJointWaypoint(start_waypoint);
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint());

        assert(is_cwp1 || is_jwp1);
        assert(is_cwp2 || is_jwp2);

        if (is_jwp1 && is_jwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::MatrixXd states = interpolate(*pre_cwp, *cur_cwp, freespace_segments);
          for (long i = 1; i < states.cols(); ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(JointWaypoint(states.col(i)),
                                                                 MoveInstructionType::FREESPACE);
            move_instruction.setPosition(states.col(i));
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_jwp2)
        {
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(*cur_jwp, MoveInstructionType::FREESPACE);
            move_instruction.setPosition(*cur_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp2 && is_jwp1)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();

          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(*pre_jwp, MoveInstructionType::FREESPACE);
            move_instruction.setPosition(*pre_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_cwp2)
        {
          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(JointWaypoint(current_jv),
                                                                 MoveInstructionType::FREESPACE);
            move_instruction.setPosition(current_jv);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          throw std::runtime_error("tesseract_planning::generateSeed: unsupported waypoints provided!");
        }

        seed.push_back(composite);
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      start_waypoint = plan_instruction->getWaypoint();
    }
    else
    {
      seed.push_back(instruction);
    }
  }
  return seed;
}

CompositeInstruction SeedGenerator::generateSeed(const CompositeInstruction& instructions)
{
  current_jv = current_state->getJointValues(fwd_kin->getJointNames());
  world_to_base = current_state->link_transforms.at(fwd_kin->getBaseLinkName());

  CompositeInstruction seed;

  // Get the start waypoint/instruction
  start_waypoint = NullWaypoint();
  MoveInstruction seed_start(start_waypoint, MoveInstructionType::START);
  if (instructions.hasStartInstruction())
  {
    assert(isMoveInstruction(instructions.getStartInstruction()));
    const auto* start_instruction = instructions.getStartInstruction().cast_const<MoveInstruction>();
    assert(start_instruction->isStart() || start_instruction->isStartFixed());
    start_waypoint = start_instruction->getWaypoint();

    seed_start.setWaypoint(start_waypoint);
    if (start_instruction->isStartFixed() && !isJointWaypoint(start_waypoint))
      throw std::runtime_error("Plan instruction with type START_FIXED must have a joint waypoint type");

    if (isJointWaypoint(start_waypoint))
      seed_start.setPosition(*(start_waypoint.cast<JointWaypoint>()));
    else if (isCartesianWaypoint(start_waypoint))
      seed_start.setPosition(current_jv);
    else
      throw std::runtime_error("Generate Seed: Unsupported waypoint type!");
  }
  else
  {
    JointWaypoint temp(current_jv);
    temp.joint_names = fwd_kin->getJointNames();
    start_waypoint = temp;

    seed_start.setPosition(current_jv);
  }
  // Process the seed
  seed = processCompositeInstruction(instructions);

  // Set start instruction
  seed.setStartInstruction(seed_start);

  return seed;
}

};  // namespace tesseract_planning
