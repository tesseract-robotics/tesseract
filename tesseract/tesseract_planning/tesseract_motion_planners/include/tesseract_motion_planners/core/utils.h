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
inline Eigen::MatrixXd interpolate(const Eigen::VectorXd& start,
                                   const Eigen::VectorXd& stop,
                                   int steps)
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
        result.push_back(CartesianWaypoint(eigen_pose));

      return result;
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      const auto* w1 = start.cast_const<Eigen::VectorXd>();
      const auto* w2 = stop.cast_const<Eigen::VectorXd>();
      Eigen::MatrixXd joint_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(joint_poses.cols());
      for ( int i = 0; i < joint_poses.cols(); ++i)
        result.push_back(JointWaypoint(joint_poses.col(i)));

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
                                         const tesseract_environment::EnvState::ConstPtr current_state,
                                         const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin,
                                         const tesseract_kinematics::InverseKinematics::Ptr& /*inv_kin*/,
                                         double /*longest_freespace_segment*/ = 0.01,
                                         double /*longest_cartesian_segment*/ = 0.01)
{
  CompositeInstruction seed;
  const PlanInstruction* prev_plan_instruction {nullptr};
  Eigen::VectorXd current_jv = current_state->getJointValues(fwd_kin->getJointNames());
  Eigen::Isometry3d world_to_base = current_state->link_transforms.at(fwd_kin->getBaseLinkName());

  for (const auto& instruction : instructions)
  {
    if (instruction.isPlan())
    {
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        CompositeInstruction composite;
        if (prev_plan_instruction)
        {
          bool is_cwp1 = isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType());
          bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
          bool is_jwp1 = isJointWaypoint(prev_plan_instruction->getWaypoint().getType());
          bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());

          assert(is_cwp1 || is_jwp1);
          assert(is_cwp2 || is_jwp2);
          assert(prev_plan_instruction->getTCP().isApprox(plan_instruction->getTCP(), 1e-5));

          if (is_cwp1 && is_cwp2)
          {
            // If both are cartesian it will cartesian interpolate and use the current state as the seed.
            const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();
            const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

            tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, *cur_cwp, 10);
            for (std::size_t p = 1; p < poses.size(); ++p)
            {
              tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
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
            const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();
            const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

            Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
            if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
              throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

            p2 = world_to_base * p2 * plan_instruction->getTCP();
            tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, p2, 10);
            for (std::size_t p = 1; p < poses.size(); ++p)
            {
              tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
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
            const auto* pre_jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
            const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

            Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
            if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
              throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

            p1 = world_to_base * p1 * prev_plan_instruction->getTCP();
            tesseract_common::VectorIsometry3d poses = interpolate(p1, *cur_cwp, 10);
            for (std::size_t p = 1; p < poses.size(); ++p)
            {
              tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
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
            const auto* pre_jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
            const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

            Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
            if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
              throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

            p1 = world_to_base * p1 * prev_plan_instruction->getTCP();

            Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
            if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
              throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

            p2 = world_to_base * p2 * plan_instruction->getTCP();
            tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, 10);
            Eigen::MatrixXd joint_poses = interpolate(*pre_jwp, *cur_jwp, 10);
            for (std::size_t p = 1; p < poses.size(); ++p)
            {
              tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
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
        }
        else
        {
          bool is_cwp = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
          bool is_jwp = isJointWaypoint(plan_instruction->getWaypoint().getType());
          assert(is_cwp || is_jwp);

          tesseract_planning::MoveInstruction move_instruction(plan_instruction->getWaypoint(), MoveInstructionType::LINEAR);

          if (is_jwp)
            move_instruction.setPosition(*(plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }
        seed.push_back(composite);
      }
      else if (plan_instruction->isFreespace())
      {
        CompositeInstruction composite;
        if (prev_plan_instruction)
        {
          bool is_cwp1 = isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType());
          bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
          bool is_jwp1 = isJointWaypoint(prev_plan_instruction->getWaypoint().getType());
          bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());

          assert(is_cwp1 || is_jwp1);
          assert(is_cwp2 || is_jwp2);

          if (is_jwp1 && is_jwp2)
          {
            const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
            const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

            Eigen::MatrixXd states = interpolate(*pre_cwp, *cur_cwp, 10);
            for (long i = 1; i < states.cols(); ++i)
            {
              tesseract_planning::MoveInstruction move_instruction(JointWaypoint(states.col(i)), MoveInstructionType::FREESPACE);
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

            for (long i = 1; i < 10; ++i)
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
            const auto* pre_jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();

            for (long i = 1; i < 10; ++i)
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
            for (long i = 1; i < 10; ++i)
            {
              tesseract_planning::MoveInstruction move_instruction(JointWaypoint(current_jv), MoveInstructionType::FREESPACE);
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
        }
        else
        {
          bool is_cwp = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
          bool is_jwp = isJointWaypoint(plan_instruction->getWaypoint().getType());
          assert(is_cwp || is_jwp);

          tesseract_planning::MoveInstruction move_instruction(plan_instruction->getWaypoint(), MoveInstructionType::FREESPACE);
          if (is_jwp)
            move_instruction.setPosition(*(plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }
        seed.push_back(composite);
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }
      prev_plan_instruction = plan_instruction;
    }
    else
    {
      seed.push_back(instruction);
    }
  }
  return seed;
}
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_PLANNING_UTILS_H
