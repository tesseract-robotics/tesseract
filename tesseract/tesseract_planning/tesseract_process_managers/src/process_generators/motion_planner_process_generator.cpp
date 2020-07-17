#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>

namespace tesseract_planning
{
MotionPlannerProcessGenerator::MotionPlannerProcessGenerator(std::shared_ptr<MotionPlanner> planner) : planner(planner)
{
  name = "motion_planner_";
}

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(
      &MotionPlannerProcessGenerator::process, this, task_inputs_.back(), null_instruction, null_instruction);
}

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input,
                                                                  const Instruction& start_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(
      &MotionPlannerProcessGenerator::process, this, task_inputs_.back(), start_instruction, null_instruction);
}

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input,
                                                                  const Instruction& start_instruction,
                                                                  const Instruction& end_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(
      &MotionPlannerProcessGenerator::process, this, task_inputs_.back(), start_instruction, end_instruction);
}

std::function<int()> MotionPlannerProcessGenerator::generateConditionalTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(&MotionPlannerProcessGenerator::conditionalProcess,
                   this,
                   task_inputs_.back(),
                   null_instruction,
                   null_instruction);
}

std::function<int()> MotionPlannerProcessGenerator::generateConditionalTask(ProcessInput input,
                                                                            const Instruction& start_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(&MotionPlannerProcessGenerator::conditionalProcess,
                   this,
                   task_inputs_.back(),
                   start_instruction,
                   null_instruction);
}

std::function<int()> MotionPlannerProcessGenerator::generateConditionalTask(ProcessInput input,
                                                                            const Instruction& start_instruction,
                                                                            const Instruction& end_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(&MotionPlannerProcessGenerator::conditionalProcess,
                   this,
                   task_inputs_.back(),
                   start_instruction,
                   end_instruction);
}

int MotionPlannerProcessGenerator::conditionalProcess(const ProcessInput& input,
                                                      const Instruction& start_instruction,
                                                      const Instruction& end_instruction) const
{
  // --------------------
  // Check that inputs are valid
  // --------------------
  if (!isCompositeInstruction(input.instruction))
  {
    CONSOLE_BRIDGE_logError("Input instructions to TrajOpt Planner must be a composite instruction");
    return 0;
  }
  if (!isCompositeInstruction(input.results))
  {
    CONSOLE_BRIDGE_logError("Input seed to TrajOpt Planner must be a composite instruction");
    return 0;
  }

  // Make a non-const copy of the input instructions to update the start/end
  CompositeInstruction instructions = *input.instruction.cast_const<CompositeInstruction>();

  // If the start and end waypoints need to be updated prior to planning
  if (!isNullInstruction(start_instruction))
  {
    // add start
    instructions.setStartInstruction(start_instruction);
  }
  if (!isNullInstruction(end_instruction))
  {
    // add end
    assert(isMoveInstruction(end_instruction));
    getLastPlanInstruction(instructions)->setWaypoint(end_instruction.cast_const<MoveInstruction>()->getWaypoint());
  }

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = *input.results.cast<CompositeInstruction>();
  request.env_state = input.tesseract->getEnvironmentConst()->getCurrentState();
  request.tesseract = input.tesseract;
  request.manipulator = "manipulator";  // TODO: Where do I get this information?
  //  request.manipulator_ik_solver;
  request.instructions = instructions;

  // --------------------
  // Fill out response
  // --------------------
  PlannerResponse response;

  bool verbose = true;
  auto status = planner->solve(request, response, verbose);

  // --------------------
  // Verify Success
  // --------------------
  // TODO: Add optional collision check
  if (status)
  {
    CONSOLE_BRIDGE_logDebug("TrajOpt Planner process succeeded");
    // Copy results back into input
    input.results = response.results;
    return 1;
  }

  CONSOLE_BRIDGE_logDebug("TrajOpt Planner process failed");
  return 0;
}

void MotionPlannerProcessGenerator::process(const ProcessInput& input,
                                            const Instruction& start_instruction,
                                            const Instruction& end_instruction) const
{
  conditionalProcess(input, start_instruction, end_instruction);
}

}  // namespace tesseract_planning
