
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_managers/simple_process_manager.h>

using namespace tesseract_planning;

SimpleProcessManager::SimpleProcessManager(TaskflowGenerator::UPtr taskflow_generator, std::size_t n)
  : taskflow_generator_(std::move(taskflow_generator)), executor_(n), taskflow_("SimpleProcessManagerTaskflow")
{
}

bool SimpleProcessManager::init(ProcessInput input)
{
  // Clear the process manager
  clear();

  // Check the overall input
  if (!isCompositeInstruction(*(input.instruction)))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should be a composite");
    return false;
  }
  const auto* composite = input.instruction->cast_const<CompositeInstruction>();

  // Check that it has a start instruction
  if (!composite->hasStartInstruction() && input.start_instruction_ptr == nullptr &&
      isNullInstruction(input.start_instruction))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should have a start instruction");
    return false;
  }

  // Create the dependency graph
  input.instruction->print("Generating Taskflow for: ");
  auto task = taskflow_
                  .composed_of(taskflow_generator_->generateTaskflow(
                      input, [this]() { successCallback(); }, [this]() { failureCallback(); }))
                  .name("Simple");
  simple_tasks_.push_back(task);

  // Dump the taskflow
  std::ofstream out_data;
  out_data.open("/tmp/simple_process_manager.dot");
  taskflow_.dump(out_data);
  out_data.close();

  return true;
}

bool SimpleProcessManager::execute()
{
  success_ = false;
  executor_.run(taskflow_).wait();
  return success_;
}

bool SimpleProcessManager::terminate()
{
  taskflow_generator_->abort();
  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool SimpleProcessManager::clear()

{
  taskflow_generator_->clear();
  taskflow_.clear();
  simple_tasks_.clear();
  return true;
}

void SimpleProcessManager::successCallback()
{
  CONSOLE_BRIDGE_logInform("SimpleProcessManager Successful");
  success_ = true;
}

void SimpleProcessManager::failureCallback()
{
  CONSOLE_BRIDGE_logInform("SimpleProcessManager Failure");
  success_ = false;
}
