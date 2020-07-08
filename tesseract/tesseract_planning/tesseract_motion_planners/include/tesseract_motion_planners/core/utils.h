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
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

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
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
/**
 * @brief Inerpolate between two transforms return a vector of Eigen::Isometry transforms.
 * @param start The Start Transform
 * @param stop The Stop/End Transform
 * @param steps The number of step
 * @return A vector of Eigen::Isometry with a length = steps + 1
 */
inline tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                                      const Eigen::Isometry3d& stop,
                                                      int steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

/**
 * @brief Inerpolate between two Eigen::VectorXd and return a Matrix
 * @param start The Start State
 * @param stop The Stop/End State
 * @param steps The number of step
 * @return A matrix where columns = steps + 1
 */
inline Eigen::MatrixXd interpolate(const Eigen::VectorXd& start, const Eigen::VectorXd& stop, int steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

/**
 * @brief Inerpolate between two waypoints return a vector of waypoints.
 * @param start The Start Waypoint
 * @param stop The Stop/End Waypoint
 * @param steps The number of step
 * @return A vector of waypoints with a length = steps + 1
 */
inline std::vector<Waypoint> interpolate(const Waypoint& start, const Waypoint& stop, int steps)
{
  switch (start.getType())
  {
    case static_cast<int>(WaypointType::CARTESIAN_WAYPOINT):
    {
      const auto* w1 = start.cast_const<Eigen::Isometry3d>();
      const auto* w2 = stop.cast_const<Eigen::Isometry3d>();
      tesseract_common::VectorIsometry3d eigen_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
        result.emplace_back(CartesianWaypoint(eigen_pose));

      return result;
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      const auto* w1 = start.cast_const<Eigen::VectorXd>();
      const auto* w2 = stop.cast_const<Eigen::VectorXd>();
      Eigen::MatrixXd joint_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(static_cast<std::size_t>(joint_poses.cols()));
      for (int i = 0; i < joint_poses.cols(); ++i)
        result.emplace_back(JointWaypoint(joint_poses.col(i)));

      return result;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType());
      return std::vector<Waypoint>();
    }
  }
}

inline CompositeInstruction generateSeed(const CompositeInstruction& instructions,
                                         const tesseract_environment::EnvState::ConstPtr& current_state,
                                         const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin,
                                         const tesseract_kinematics::InverseKinematics::Ptr& /*inv_kin*/,
                                         int freespace_segments,
                                         int cartesian_segments)
{
  CompositeInstruction seed;
  Eigen::VectorXd current_jv = current_state->getJointValues(fwd_kin->getJointNames());
  Eigen::Isometry3d world_to_base = current_state->link_transforms.at(fwd_kin->getBaseLinkName());

  Waypoint start_waypoint = NullWaypoint();
  if (instructions.hasStartWaypoint())
  {
    start_waypoint = instructions.getStartWaypoint();
  }
  else
  {
    JointWaypoint temp(current_jv);
    temp.joint_names = fwd_kin->getJointNames();
    start_waypoint = temp;
  }

  bool found_plan_instruction = false;
  for (const auto& instruction : instructions)
  {
    if (instruction.isPlan())
    {
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        CompositeInstruction composite;

        bool is_cwp1 = isCartesianWaypoint(start_waypoint.getType());
        bool is_jwp1 = isJointWaypoint(start_waypoint.getType());
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());
        if (!found_plan_instruction)
        {
          tesseract_planning::MoveInstruction move_instruction(start_waypoint,
                                                               MoveInstructionType::LINEAR);

          if (is_jwp1)
            move_instruction.setPosition(*(start_waypoint.cast_const<JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }

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

        bool is_cwp1 = isCartesianWaypoint(start_waypoint.getType());
        bool is_jwp1 = isJointWaypoint(start_waypoint.getType());
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());
        if (!found_plan_instruction)
        {
          tesseract_planning::MoveInstruction move_instruction(start_waypoint,
                                                               MoveInstructionType::FREESPACE);

          if (is_jwp1)
            move_instruction.setPosition(*(start_waypoint.cast_const<JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }

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

      found_plan_instruction = true;
      start_waypoint = plan_instruction->getWaypoint();
    }
    else
    {
      seed.push_back(instruction);
    }
  }

  return seed;
}

inline CompositeInstruction generateSeed(const CompositeInstruction& instructions,
                                         const tesseract_environment::EnvState::ConstPtr& current_state,
                                         const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin,
                                         const tesseract_kinematics::InverseKinematics::Ptr& inv_kin)
{
  return generateSeed(instructions, current_state, fwd_kin, inv_kin, 10, 10);
}

/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 */
inline void FlattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened, CompositeInstruction& composite)
{
  for (auto& i : composite)
  {
    if (i.isComposite())
      FlattenHelper(flattened, *(i.cast<CompositeInstruction>()));
    else
      flattened.emplace_back(i);
  }
}

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 * @param instruction Input composite instruction to be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<Instruction>> Flatten(CompositeInstruction& instruction)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  FlattenHelper(flattened, instruction);
  return flattened;
}

/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 */
inline void FlattenHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                          const CompositeInstruction& composite)
{
  for (auto& i : composite)
  {
    if (i.isComposite())
      FlattenHelper(flattened, *(i.cast_const<CompositeInstruction>()));
    else
      flattened.emplace_back(i);
  }
}

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 * @param instruction Input composite instruction to be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<const Instruction>> Flatten(const CompositeInstruction& instruction)
{
  std::vector<std::reference_wrapper<const Instruction>> flattened;
  FlattenHelper(flattened, instruction);
  return flattened;
}

inline void FlattenToPatternHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                                   CompositeInstruction& composite,
                                   const CompositeInstruction& pattern)
{
  if (composite.size() != pattern.size())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  for (std::size_t i = 0; i < pattern.size(); i++)
  {
    if (pattern.at(i).isComposite() && composite[i].isComposite())
      FlattenToPatternHelper(
          flattened, *(composite[i].cast<CompositeInstruction>()), *pattern.at(i).cast_const<CompositeInstruction>());
    else
      flattened.emplace_back(composite[i]);
  }
}

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<Instruction>> FlattenToPattern(CompositeInstruction& instruction,
                                                                         const CompositeInstruction& pattern)
{
  if (instruction.size() != pattern.size())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return std::vector<std::reference_wrapper<Instruction>>();
  }

  std::vector<std::reference_wrapper<Instruction>> flattened;
  FlattenToPatternHelper(flattened, instruction, pattern);
  return flattened;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_UTILS_H
