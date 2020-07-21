
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

FreespaceProcessManager::FreespaceProcessManager() : taskflow("FreespaceProcessManagerTaskflow") {}

bool FreespaceProcessManager::init(ProcessInput input)
{
  // Clear the taskflow
  taskflow.clear();

  // If no processes selected, use defaults
  if (process_generators.empty())
    process_generators = defaultFreespaceProcesses();

  // Create the taskflow generator
  taskflow_generator = SequentialFailureTreeTaskflow(process_generators);

  // Create the dependency graph
  assert(isCompositeInstruction(input.instruction));

  input.instruction.print("Generating Taskflow for: ");
  auto task = taskflow
                  .composed_of(taskflow_generator.generateTaskflow(
                      input, [this]() { successCallback(); }, [this]() { failureCallback(); }))
                  .name("freespace");
  freespace_tasks.push_back(task);

  // Dump the taskflow
  std::ofstream out_data;
  out_data.open("freespace_process_manager.dot");
  taskflow.dump(out_data);
  out_data.close();

  return true;
}

bool FreespaceProcessManager::execute()
{
  success = false;
  executor.run(taskflow).wait();
  return success;
}

bool FreespaceProcessManager::terminate()
{
  CONSOLE_BRIDGE_logError("Terminate is not implemented");
  return false;
}

bool FreespaceProcessManager::clear()

{
  taskflow.clear();
  freespace_tasks.clear();
  return true;
}

void FreespaceProcessManager::successCallback()
{
  CONSOLE_BRIDGE_logInform("FreespaceProcessManager Successful");
  success = true;
}

void FreespaceProcessManager::failureCallback()
{
  CONSOLE_BRIDGE_logInform("FreespaceProcessManager Failure");
  success = false;
}
