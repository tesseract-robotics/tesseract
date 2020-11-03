/**
 * @file raster_example_program.h
 * @brief Example raster paths
 *
 * @author Levi Armstrong
 * @date August 28, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_RASTER_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGERS_RASTER_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/command_language.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
inline CompositeInstruction rasterExampleProgram(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                 const std::string& process_profile = "PROCESS")
{
  CompositeInstruction program(DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));

  // Define from start composite instruction
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start(freespace_profile);
  from_start.setDescription("from_start");
  from_start.push_back(plan_f0);
  program.push_back(from_start);

  //
  for (int i = 0; i < 4; ++i)
  {
    double x = 0.8 + (i * 0.1);
    Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.0, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));

    CompositeInstruction raster_segment(process_profile);
    raster_segment.setDescription("Raster #" + std::to_string(i + 1));
    if (i == 0 || i == 2)
    {
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp7, PlanInstructionType::LINEAR, process_profile));
    }
    else
    {
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp1, PlanInstructionType::LINEAR, process_profile));
    }
    program.push_back(raster_segment);

    // Add transition
    if (i == 0 || i == 2)
    {
      Waypoint wp7 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), 0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp7, PlanInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("transition_from_end");
      transition.push_back(plan_f1);
      program.push_back(transition);
    }
    else if (i == 1)
    {
      Waypoint wp1 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), -0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("transition_from_end");
      transition.push_back(plan_f1);
      program.push_back(transition);
    }
  }

  PlanInstruction plan_f2(swp1, PlanInstructionType::FREESPACE, freespace_profile);
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end(freespace_profile);
  to_end.setDescription("to_end");
  to_end.push_back(plan_f2);
  program.push_back(to_end);

  return program;
}

inline CompositeInstruction rasterOnlyExampleProgram(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                     const std::string& process_profile = "PROCESS")
{
  CompositeInstruction program(DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));

  // Define start instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START);
  start_instruction.setDescription("Start Instruction");
  program.setStartInstruction(start_instruction);

  for (int i = 0; i < 4; ++i)
  {
    double x = 0.8 + (i * 0.1);
    Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.0, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));

    CompositeInstruction raster_segment(process_profile);
    raster_segment.setDescription("Raster #" + std::to_string(i + 1));
    if (i == 0 || i == 2)
    {
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp7, PlanInstructionType::LINEAR, process_profile));
    }
    else
    {
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, process_profile));
      raster_segment.push_back(PlanInstruction(wp1, PlanInstructionType::LINEAR, process_profile));
    }
    program.push_back(raster_segment);

    // Add transition
    if (i == 0 || i == 2)
    {
      Waypoint wp7 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), 0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp7, PlanInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("transition_from_end");
      transition.push_back(plan_f1);
      program.push_back(transition);
    }
    else if (i == 1)
    {
      Waypoint wp1 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), -0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("transition_from_end");
      transition.push_back(plan_f1);
      program.push_back(transition);
    }
  }

  return program;
}

}  // namespace tesseract_planning

#endif
