#ifndef TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
inline CompositeInstruction ExampleProgram()
{
  CompositeInstruction program;
  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define raster poses
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR, "cartesian_profile");

  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("to_start");
  program.push_back(plan_f0);

  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f1.setDescription("freespace");
  program.push_back(plan_f1);

  CompositeInstruction raster1;
  raster1.setDescription("raster");
  raster1.push_back(plan_c1);
  raster1.push_back(plan_c2);
  raster1.push_back(plan_c3);
  raster1.push_back(plan_c4);
  raster1.push_back(plan_c5);
  program.push_back(raster1);

  PlanInstruction plan_f2(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f2.setDescription("freespace");
  program.push_back(plan_f2);

  CompositeInstruction raster2;
  raster2.setDescription("raster");
  raster2.push_back(plan_c1);
  raster2.push_back(plan_c2);
  raster2.push_back(plan_c3);
  raster2.push_back(plan_c4);
  raster2.push_back(plan_c5);
  program.push_back(raster2);

  PlanInstruction plan_f3(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f3.setDescription("freespace");
  program.push_back(plan_f3);

  CompositeInstruction raster3;
  raster3.setDescription("raster");
  raster3.push_back(plan_c1);
  raster3.push_back(plan_c2);
  raster3.push_back(plan_c3);
  raster3.push_back(plan_c4);
  raster3.push_back(plan_c5);
  program.push_back(raster3);

  PlanInstruction plan_f4(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f4.setDescription("freespace");
  program.push_back(plan_f4);

  return program;
}

}  // namespace tesseract_planning

#endif
