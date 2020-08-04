#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>

namespace tesseract_planning
{
MotionPlannerProcessGenerator::MotionPlannerProcessGenerator(std::shared_ptr<MotionPlanner> planner) : planner(planner)
{
  name = planner->getName();
}

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(
      &MotionPlannerProcessGenerator::process, this, task_inputs_.back(), null_instruction_, null_instruction_);
}

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input,
                                                                  const Instruction& start_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(
      &MotionPlannerProcessGenerator::process, this, task_inputs_.back(), start_instruction, null_instruction_);
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
                   null_instruction_,
                   null_instruction_);
}

std::function<int()> MotionPlannerProcessGenerator::generateConditionalTask(ProcessInput input,
                                                                            const Instruction& start_instruction)
{
  task_inputs_.push_back(input);

  return std::bind(&MotionPlannerProcessGenerator::conditionalProcess,
                   this,
                   task_inputs_.back(),
                   start_instruction,
                   null_instruction_);
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
  if (abort_)
    return 0;
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
  if (instructions.getManipulatorInfo().isEmpty())
  {
    assert(!input.manip_info.isEmpty());
    instructions.setManipulatorInfo(input.manip_info);
  }

  // If the start and end waypoints need to be updated prior to planning
  if (!isNullInstruction(start_instruction))
  {
    // add start
    if (isCompositeInstruction(start_instruction))
    {
      // if provided a composite instruction as the start instruction it will extract the last move instruction
      const auto* ci = start_instruction.cast_const<CompositeInstruction>();
      auto* lmi = getLastMoveInstruction(*ci);
      assert(lmi != nullptr);
      assert(isMoveInstruction(*lmi));
      PlanInstruction si(lmi->getWaypoint(), PlanInstructionType::START, lmi->getProfile(), lmi->getManipulatorInfo());
      instructions.setStartInstruction(si);
    }
    else
    {
      assert(isPlanInstruction(start_instruction));
      instructions.setStartInstruction(start_instruction);
      instructions.getStartInstruction().cast<PlanInstruction>()->setPlanType(PlanInstructionType::START);
    }
  }
  if (!isNullInstruction(end_instruction))
  {
    // add end
    if (isCompositeInstruction(end_instruction))
    {
      // if provided a composite instruction as the end instruction it will extract the first move instruction
      const auto* ci = end_instruction.cast_const<CompositeInstruction>();
      auto* fmi = getFirstMoveInstruction(*ci);
      assert(fmi != nullptr);
      assert(isMoveInstruction(*fmi));
      getLastPlanInstruction(instructions)->setWaypoint(fmi->getWaypoint());
    }
    else
    {
      assert(isMoveInstruction(end_instruction));
      auto* lpi = getLastPlanInstruction(instructions);
      lpi->setWaypoint(end_instruction.cast_const<MoveInstruction>()->getWaypoint());
    }
  }

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = *input.results.cast<CompositeInstruction>();
  request.env_state = input.tesseract->getEnvironmentConst()->getCurrentState();
  request.tesseract = input.tesseract;
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
  if (status)
  {
    CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
    int success = 1;
    for (auto validator : validators)
    {
      ProcessInput validate_input = input;
      validate_input.results = response.results;
      success &= validator(validate_input);
    }
    if (success == 1)
    {
      input.results = response.results;
      CONSOLE_BRIDGE_logDebug("Motion Planner process results passed validators.");
      return 1;
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("Motion Planner process results did not pass validators.");
    }
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Motion Planner process failed");
  }

  return 0;
}

void MotionPlannerProcessGenerator::process(const ProcessInput& input,
                                            const Instruction& start_instruction,
                                            const Instruction& end_instruction) const
{
  conditionalProcess(input, start_instruction, end_instruction);
}

bool MotionPlannerProcessGenerator::getAbort() const { return abort_; }
void MotionPlannerProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
