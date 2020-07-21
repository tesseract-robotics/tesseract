/**
 * @file plan_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
enum class PlanInstructionType : int
{
  LINEAR,
  FREESPACE,
  CIRCULAR
};

class PlanInstruction
{
public:
  using Ptr = std::shared_ptr<PlanInstruction>;
  using ConstPtr = std::shared_ptr<const PlanInstruction>;

  PlanInstruction(Waypoint waypoint,
                  PlanInstructionType type,
                  std::string profile = "DEFAULT",
                  std::string working_frame = "");

  void setWaypoint(Waypoint waypoint);
  const Waypoint& getWaypoint() const;

  void setTCP(Eigen::Isometry3d tcp);
  const Eigen::Isometry3d& getTCP() const;

  void setWorkingFrame(std::string working_frame);
  const std::string& getWorkingFrame() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(std::string prefix = "") const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

private:
  int type_{ static_cast<int>(InstructionType::PLAN_INSTRUCTION) };

  PlanInstructionType plan_type_;

  /** @brief The assigned waypoint (Cartesian or Joint) */
  Waypoint waypoint_;

  /** @brief The profile used for this plan instruction */
  std::string profile_{ "DEFAULT" };

  /** @brief The tool center point */
  Eigen::Isometry3d tcp_{ Eigen::Isometry3d::Identity() };

  /** @brief The working frame the waypoint is relative to */
  std::string working_frame_;

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Plan Instruction" };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
