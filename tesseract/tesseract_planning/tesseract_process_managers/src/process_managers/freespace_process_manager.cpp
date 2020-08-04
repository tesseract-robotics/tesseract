
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_managers/freespace_process_manager.h>

#include <taskflow/taskflow.hpp>
#include <fstream>
#include <tesseract_process_managers/process_generators/random_process_generator.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/taskflow_generators/sequential_failure_tree_taskflow.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_process_managers/process_managers/default_processes/default_freespace_processes.h>

using namespace tesseract_planning;

FreespaceProcessManager::FreespaceProcessManager(std::size_t n)
  : executor_(n)
  , taskflow_("FreespaceProcessManagerTaskflow")
{}

bool FreespaceProcessManager::init(ProcessInput input)
{
  // Clear the taskflow
  taskflow_.clear();

  // If no processes selected, use defaults
  if (process_generators.empty())
    process_generators = defaultFreespaceProcesses();

  // Create the taskflow generator
  taskflow_generator_ = SequentialFailureTreeTaskflow(process_generators);

  // Create the dependency graph
  assert(isCompositeInstruction(input.instruction));

  input.instruction.print("Generating Taskflow for: ");
  auto task = taskflow_
                  .composed_of(taskflow_generator_.generateTaskflow(
                      input, [this]() { successCallback(); }, [this]() { failureCallback(); }))
                  .name("freespace");
  freespace_tasks_.push_back(task);

  // Dump the taskflow
  std::ofstream out_data;
  out_data.open("/tmp/freespace_process_manager.dot");
  taskflow_.dump(out_data);
  out_data.close();

  return true;
}

bool FreespaceProcessManager::execute()
{
  success_ = false;
  executor_.run(taskflow_).wait();
  return success_;
}

bool FreespaceProcessManager::terminate()
{
  for (auto gen : process_generators)
    gen->setAbort(true);

  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool FreespaceProcessManager::clear()

{
  for (auto gen : process_generators)
    gen->setAbort(false);
  taskflow_.clear();
  freespace_tasks_.clear();
  return true;
}

void FreespaceProcessManager::successCallback()
{
  CONSOLE_BRIDGE_logInform("FreespaceProcessManager Successful");
  success_ = true;
}

void FreespaceProcessManager::failureCallback()
{
  CONSOLE_BRIDGE_logInform("FreespaceProcessManager Failure");
  success_ = false;
}
