#ifndef TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H

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
inline CompositeInstruction freespaceExampleProgram()
{
  CompositeInstruction program("freespace_composite");

  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(7));
  PlanInstruction start_instruction(wp1, PlanInstructionType::START_FIXED);
  program.setStartInstruction(start_instruction);

  // Define target pose
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE, "DEFAULT");
  plan_f0.setDescription("freespace_motion");
  program.push_back(plan_f0);

  return program;
}

}  // namespace tesseract_planning

#endif
