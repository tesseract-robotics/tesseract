/**
 * @file move_instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
enum class MoveInstructionType : int
{
  LINEAR,
  FREESPACE,
  CIRCULAR,
  /**< This indicates it is a start instruction and waypoint information should be used. */
  START,
  /**< This indicates that the data in position, velocity, acceleration and effort should be used instead of waypoint */
  START_FIXED
};

class MoveInstruction
{
public:
  using Ptr = std::shared_ptr<MoveInstruction>;
  using ConstPtr = std::shared_ptr<const MoveInstruction>;

  MoveInstruction(Waypoint waypoint, MoveInstructionType type, const std::string& profile = "DEFAULT");

  void setWaypoint(Waypoint waypoint);
  const Waypoint& getWaypoint() const;

  void setTCP(const Eigen::Isometry3d& tcp);
  const Eigen::Isometry3d& getTCP() const;

  void setWorkingFrame(std::string working_frame);
  const std::string& getWorkingFrame() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  void setPosition(const Eigen::VectorXd& position);
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(const Eigen::VectorXd& velocity);
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(const Eigen::VectorXd& acceleration);
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(const Eigen::VectorXd& effort);
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  const double& getTime() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(std::string prefix = "") const;

  void setMoveType(MoveInstructionType move_type);
  MoveInstructionType getMoveType() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

  bool isStart() const;

  bool isStartFixed() const;

private:
  int type_{ static_cast<int>(InstructionType::MOVE_INSTRUCTION) };

  MoveInstructionType move_type_;
  std::string description_;

  /** @brief The profile used for this move instruction */
  std::string profile_{ "DEFAULT" };

  /** @brief The assigned waypoint (Cartesian or Joint) */
  Waypoint waypoint_;

  /** @brief The tool center point */
  Eigen::Isometry3d tcp_{ Eigen::Isometry3d::Identity() };

  /** @brief The working frame the waypoint is relative to */
  std::string working_frame_;

  /**
   * @brief The joint position at the waypoint
   *
   * This is different from waypoint because it can be cartesian or joint and this stores the joint position solved
   * for the planned waypoint. This also can be used for determining the robot configuration if provided a cartesian
   * waypoint for generating a native robot program.
   */
  Eigen::VectorXd position_;

  /** @brief The velocity at the waypoint */
  Eigen::VectorXd velocity_;

  /** @brief The Acceleration at the waypoint */
  Eigen::VectorXd acceleration_;

  /** @brief The Effort at the waypoint */
  Eigen::VectorXd effort_;

  /** @brief The Time from start at the waypoint */
  double time_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
