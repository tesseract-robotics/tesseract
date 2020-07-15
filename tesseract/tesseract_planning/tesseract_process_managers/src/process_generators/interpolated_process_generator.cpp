#include <tesseract_process_managers/process_generators/interpolated_process_generator.h>

namespace tesseract_planning
{
std::function<void()> InterpolatedProcessGenerator::generateTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(&InterpolatedProcessGenerator::process, this, task_inputs_.back());
}

std::function<int()> InterpolatedProcessGenerator::generateConditionalTask(ProcessInput input)
{
  task_inputs_.push_back(input);

  return std::bind(&InterpolatedProcessGenerator::conditionalProcess, this, task_inputs_.back());
}

int InterpolatedProcessGenerator::conditionalProcess(ProcessInput /*input*/) const { return 0; }

void InterpolatedProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

}  // namespace tesseract_planning
