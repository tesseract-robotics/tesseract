
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>

#include <tesseract_process_managers/process_generators/random_process_generator.h>
#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/taskflow_generators/sequential_failure_tree_taskflow.h>
#include <tesseract_process_managers/process_managers/default_processes/default_freespace_processes.h>
#include <tesseract_process_managers/process_managers/default_processes/default_raster_processes.h>
#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_command_language/command_language_utils.h>

using namespace tesseract_planning;

struct BasicObserver : public tf::ObserverInterface
{
  BasicObserver(const std::string& name) { std::cout << "Constructing observer " << name << '\n'; }

  void set_up(size_t num_workers) override final
  {
    std::cout << "Setting up observer with " << num_workers << " workers\n";
  }

  void on_entry(size_t w, tf::TaskView tv) override final
  {
    std::ostringstream oss;
    oss << "worker " << w << " ready to run " << tv.name() << '\n';
    std::cout << oss.str();
  }

  void on_exit(size_t w, tf::TaskView tv) override final
  {
    std::ostringstream oss;
    oss << "worker " << w << " finished running " << tv.name() << '\n';
    std::cout << oss.str();
  }
};

RasterProcessManager::RasterProcessManager(TaskflowGenerator::UPtr freespace_taskflow_generator,
                                           TaskflowGenerator::UPtr raster_taskflow_generator,
                                           std::size_t n)
  : freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , executor_(n)
  , taskflow_("RasterProcessManagerTaskflow")
{
}

bool RasterProcessManager::init(ProcessInput input)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    return false;
  }

  // Clear the taskflow
  taskflow_.clear();

  // Store the current size of the tasks so that we can add from_start later
  std::size_t starting_raster_idx = raster_tasks_.size();

  // Generate all of the raster tasks. They don't depend on anything
  for (std::size_t idx = 1; idx < input.size() - 1; idx += 2)
  {
    // Rasters can have multiple steps (e.g. approach, process, departure), but they are all flattened

    // Get Start Plan Instruction
    Instruction start_instruction = NullInstruction();
    if (idx == 1)
    {
      assert(isCompositeInstruction(*(input[0].instruction)));
      const auto* ci = input[0].instruction->cast_const<CompositeInstruction>();
      const auto* li = getLastPlanInstruction(*ci);
      assert(li != nullptr);
      start_instruction = *li;
    }
    else
    {
      assert(isCompositeInstruction(*(input[idx - 1].instruction)));
      const auto* tci = input[idx - 1].instruction->cast_const<CompositeInstruction>();
      assert(isCompositeInstruction((*tci)[0]));
      const auto* ci = (*tci)[0].cast_const<CompositeInstruction>();
      auto* li = getLastPlanInstruction(*ci);
      assert(li != nullptr);
      start_instruction = *li;
    }

    start_instruction.cast<PlanInstruction>()->setPlanType(PlanInstructionType::START);
    ProcessInput raster_input = input[idx];
    raster_input.start_instruction = start_instruction;
    auto raster_step = taskflow_
                           .composed_of(raster_taskflow_generator_->generateTaskflow(
                               raster_input,
                               std::bind(&RasterProcessManager::successCallback, this),
                               std::bind(&RasterProcessManager::failureCallback, this)))
                           .name("raster_" + std::to_string(idx));
    raster_tasks_.push_back(raster_step);
  }

  // Loop over all transitions
  std::size_t transition_idx = 0;
  for (std::size_t input_idx = 2; input_idx < input.size() - 2; input_idx += 2)
  {
    // Each transition step depends on the start and end only since they are independent
    for (std::size_t transition_step_idx = 0; transition_step_idx < input[input_idx].size(); transition_step_idx++)
    {
      // This use to extract the start and end, but things were changed so the seed is generated as part of the
      // taskflow. So the seed is only a skeleton and does not contain move instructions. So instead we provide the
      // composite and let the generateTaskflow extract the start and end waypoint from the composite. This is also more
      // robust because planners could modify composite size, which is rare but does happen when using OMPL where it is
      // not possible to simplify the trajectory to the desired number of states.
      ProcessInput transition_input = input[input_idx][transition_step_idx];
      transition_input.start_instruction_ptr = input[input_idx - 1].results;
      transition_input.end_instruction_ptr = input[input_idx + 1].results;
      auto transition_step =
          taskflow_
              .composed_of(freespace_taskflow_generator_->generateTaskflow(
                  transition_input,
                  std::bind(&RasterProcessManager::successCallback, this),
                  std::bind(&RasterProcessManager::failureCallback, this)))
              .name("transition_" + std::to_string(input_idx) + "." + std::to_string(transition_step_idx));

      // Each transition is independent and thus depends only on the adjacent rasters
      transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx]);
      transition_step.succeed(raster_tasks_[starting_raster_idx + transition_idx + 1]);

      freespace_tasks_.push_back(transition_step);
    }
    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  ProcessInput from_start_input = input[0];
  from_start_input.start_instruction = input.instruction->cast_const<CompositeInstruction>()->getStartInstruction();
  from_start_input.end_instruction_ptr = input[1].results;
  auto from_start = taskflow_
                        .composed_of(freespace_taskflow_generator_->generateTaskflow(
                            from_start_input,
                            std::bind(&RasterProcessManager::successCallback, this),
                            std::bind(&RasterProcessManager::failureCallback, this)))
                        .name("from_start");
  raster_tasks_[starting_raster_idx].precede(from_start);
  freespace_tasks_.push_back(from_start);

  // Plan to_end - preceded by the last raster
  ProcessInput to_end_input = input[input.size() - 1];
  to_end_input.start_instruction_ptr = input[input.size() - 2].results;
  auto to_end = taskflow_
                    .composed_of(freespace_taskflow_generator_->generateTaskflow(
                        to_end_input,
                        std::bind(&RasterProcessManager::successCallback, this),
                        std::bind(&RasterProcessManager::failureCallback, this)))
                    .name("to_end");
  raster_tasks_.back().precede(to_end);
  freespace_tasks_.push_back(to_end);

  // visualizes the taskflow
  std::ofstream out_data;
  out_data.open("/tmp/raster_process_manager.dot");
  taskflow_.dump(out_data);
  out_data.close();

  return true;
}

bool RasterProcessManager::execute()
{
  success_ = true;

  auto observer = executor_.make_observer<BasicObserver>("BasicObserver");

  // TODO: Figure out how to cancel execution. This callback is only checked at beginning of the taskflow (ie before
  // restarting)
  //  executor.run_until(taskflow, [this]() { std::cout << "Checking if done: " << this->done << std::endl; return
  //  this->done;});

  // Wait for currently running taskflows to end.
  executor_.wait_for_all();
  executor_.run(taskflow_).wait();

  return success_;
}

bool RasterProcessManager::terminate()
{
  freespace_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();

  CONSOLE_BRIDGE_logError("Terminating Taskflow");
  return false;
}

bool RasterProcessManager::clear()

{
  freespace_taskflow_generator_->reset();
  raster_taskflow_generator_->reset();
  taskflow_.clear();
  freespace_tasks_.clear();
  raster_tasks_.clear();
  return true;
}

bool RasterProcessManager::checkProcessInput(const tesseract_planning::ProcessInput& input) const
{
  // -------------
  // Check Input
  // -------------
  if (!input.tesseract)
  {
    CONSOLE_BRIDGE_logError("ProcessInput tesseract is a nullptr");
    return false;
  }

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

  // Check from_start
  if (!isCompositeInstruction(composite->at(0)))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: from_start should be a composite");
    return false;
  }

  // Check rasters and transitions
  for (std::size_t index = 1; index < composite->size() - 1; index++)
  {
    // Both rasters and transitions should be a composite
    if (!isCompositeInstruction(composite->at(index)))
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: Both rasters and transitions should be a composite");
      return false;
    }

    // Convert to composite
    const auto* step = composite->at(index).cast_const<CompositeInstruction>();

    // Odd numbers are raster segments
    if (index % 2 == 1)
    {
      // Raster must have at least one element but 3 is not enforced.
      if (!step->size())
      {
        CONSOLE_BRIDGE_logError("ProcessInput Invalid: Rasters must have at least one element");
        return false;
      }
      //      for (const auto& raster_step : step)
      //      {
      // TODO: Disabling this for now since we are flattening it. I'm not sure if this should be a requirement or not
      //        // However, all steps of the raster must be composites
      //        if (!isCompositeInstruction(raster_step))
      //        {
      //          CONSOLE_BRIDGE_logError("ProcessInput Invalid: All steps of a raster should be a composite");
      //          return false;
      //        }
      //      }
    }
    // Evens are transitions
    else
    {
      // If there is only one transition, we assume it is transition_from_end
      if (step->size() > 1)
      {
        // If there are multiple, then they should be unordered
        if (step->getOrder() != CompositeInstructionOrder::UNORDERED)
        {
          // If you get this error, check that this is not processing a raster strip. You may be missing from_start.
          CONSOLE_BRIDGE_logError("Raster contains multiple transitions but is not marked UNORDERED");
          step->print();
          return false;
        }
      }
    }
  }
  // Check to_end
  if (!isCompositeInstruction(composite->back()))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: to_end should be a composite");
    return false;
  };

  return true;
}

void RasterProcessManager::successCallback()
{
  CONSOLE_BRIDGE_logInform("Task Successful");
  success_ &= true;
}

void RasterProcessManager::failureCallback()
{
  // For this process, any failure of a sub-TaskFlow indicates a planning failure. Abort all future tasks
  freespace_taskflow_generator_->abort();
  raster_taskflow_generator_->abort();
  // Print an error if this is the first failure
  if (success_)
    CONSOLE_BRIDGE_logError("RasterProcessManager Failure");
  success_ = false;
}
