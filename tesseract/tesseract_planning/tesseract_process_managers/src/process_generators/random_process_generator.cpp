#include <tesseract_process_managers/process_generators/random_process_generator.h>

namespace tesseract_planning
{
const std::string& RandomProcessGenerator::getName() const { return name_; }

std::function<void()> RandomProcessGenerator::generateTask(ProcessInput input)
{
  return std::bind(&RandomProcessGenerator::process, this, input);
}

std::function<int()> RandomProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return std::bind(&RandomProcessGenerator::conditionalProcess, this, input);
}

void RandomProcessGenerator::process(const ProcessInput& /*results*/) const { std::cout << name_ + "\n"; }

int RandomProcessGenerator::conditionalProcess(const ProcessInput& /*results*/) const
{
  Eigen::MatrixX2d limits(1, 2);
  limits << 0, 1;
  Eigen::VectorXd rand = tesseract_common::generateRandomNumber(limits);

  int success = (rand[0] > success_frequency_) ? 0 : 1;
  std::cout << name_ + "  Success: " + std::to_string(success) + "\n";
  return success;
}

bool RandomProcessGenerator::getAbort() const { return abort_; }
void RandomProcessGenerator::setAbort(bool abort) { abort_ = abort; }
}  // namespace tesseract_planning
