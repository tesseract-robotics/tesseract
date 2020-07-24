/**
 * @file manipulator_info.h
 * @brief
 *
 * @author Levi Armstrong
 * @date July 22, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_MANIPULATOR_INFO_H
#define TESSERACT_COMMAND_LANGUAGE_MANIPULATOR_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
/**
 * @brief Contains information about a robot manipulator
 */
struct ManipulatorInfo
{
  ManipulatorInfo() = default;
  ManipulatorInfo(std::string manipulator) : manipulator(std::move(manipulator)) {}

  /** @brief Name of the manipulator group */
  std::string manipulator;

  /** @brief (Optional) IK Solver to be used */
  std::string manipulator_ik_solver;

  /** @brief (Optional) The tool center point */
  Eigen::Isometry3d tcp{ Eigen::Isometry3d::Identity() };

  /** @brief (Optional) The working frame to which waypoints are relative. If empty the base link of the environment is
   * used*/
  std::string working_frame;

  /**
   * @brief Returns true if required members are filled out
   * @return Returns true if required members are filled out
   */
  bool isEmpty() { return manipulator.empty(); }
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
