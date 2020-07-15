#include <tesseract_process_managers/process_generators/trajopt_process_generator.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>

namespace tesseract_planning
{
TrajOptProcessGenerator::TrajOptProcessGenerator()
{
  name = "trajopt";
  planner.plan_profiles["DEFAULT"] = std::make_shared<TrajOptDefaultPlanProfile>();
  planner.composite_profiles["DEFAULT"] = std::make_shared<TrajOptDefaultCompositeProfile>();
  planner.problem_generator = &DefaultTrajoptProblemGenerator;
}

std::function<void()> TrajOptProcessGenerator::generateTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(&TrajOptProcessGenerator::process, this, task_inputs_.back());
}

std::function<int()> TrajOptProcessGenerator::generateConditionalTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(&TrajOptProcessGenerator::conditionalProcess, this, task_inputs_.back());
}

int TrajOptProcessGenerator::conditionalProcess(ProcessInput input) const
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

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = *input.results.cast<CompositeInstruction>();
  request.env_state = input.tesseract->getEnvironmentConst()->getCurrentState();
  request.tesseract = input.tesseract;
  request.manipulator = "manipulator";  // TODO
  //  request.manipulator_ik_solver;
  request.instructions = *input.instruction.cast_const<CompositeInstruction>();

  // --------------------
  // Fill out response
  // --------------------
  PlannerResponse response;

  bool verbose = true;
  auto status = planner.solve(request, response, verbose);

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

void TrajOptProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

}  // namespace tesseract_planning
