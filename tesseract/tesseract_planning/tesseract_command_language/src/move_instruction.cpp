/**
 * @file move_instruction.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/move_instruction.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint, MoveInstructionType type, const std::string& profile)
  : move_type_(type), profile_(profile), waypoint_(std::move(waypoint))
{
}

void MoveInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

void MoveInstruction::setTCP(const Eigen::Isometry3d& tcp) { tcp_ = tcp; }
const Eigen::Isometry3d& MoveInstruction::getTCP() const { return tcp_; }

void MoveInstruction::setWorkingFrame(std::string working_frame) { working_frame_ = working_frame; }
const std::string& MoveInstruction::getWorkingFrame() const { return working_frame_; }

void MoveInstruction::setProfile(const std::string& profile) { profile_ = (profile.empty()) ? "DEFAULT" : profile; }
const std::string& MoveInstruction::getProfile() const { return profile_; }

void MoveInstruction::setPosition(const Eigen::VectorXd& position) { position_ = position; }
const Eigen::VectorXd& MoveInstruction::getPosition() const { return position_; }

void MoveInstruction::setVelocity(const Eigen::VectorXd& velocity) { velocity_ = velocity; }
const Eigen::VectorXd& MoveInstruction::getVelocity() const { return velocity_; }

void MoveInstruction::setAcceleration(const Eigen::VectorXd& acceleration) { acceleration_ = acceleration; }
const Eigen::VectorXd& MoveInstruction::getAcceleration() const { return acceleration_; }

void MoveInstruction::setEffort(const Eigen::VectorXd& effort) { effort_ = effort; }
const Eigen::VectorXd& MoveInstruction::getEffort() const { return effort_; }

void MoveInstruction::setTime(double time) { time_ = time; }
const double& MoveInstruction::getTime() const { return time_; }

int MoveInstruction::getType() const { return type_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

void MoveInstruction::print(std::string prefix) const
{
  std::cout << prefix + "Move Instruction, Type: " << getType() << "  Waypoint Type:" << getWaypoint().getType()
            << "  Description: " << getDescription() << std::endl;
}

void MoveInstruction::setMoveType(MoveInstructionType move_type) { move_type_ = move_type; }
MoveInstructionType MoveInstruction::getMoveType() const { return move_type_; }

bool MoveInstruction::isLinear() const { return (move_type_ == MoveInstructionType::LINEAR); }

bool MoveInstruction::isFreespace() const { return (move_type_ == MoveInstructionType::FREESPACE); }

bool MoveInstruction::isCircular() const { return (move_type_ == MoveInstructionType::CIRCULAR); }

bool MoveInstruction::isStart() const { return (move_type_ == MoveInstructionType::START); }

bool MoveInstruction::isStartFixed() const { return (move_type_ == MoveInstructionType::START_FIXED); }

}  // namespace tesseract_planning
