/**
 * @file process_planning_server.cpp
 * @brief A process planning server with a default set of process planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/debug_observer.h>
#include <tesseract_process_managers/core/default_process_planners.h>

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
ProcessPlanningServer::ProcessPlanningServer(EnvironmentCache::Ptr cache, size_t n)
  : cache_(std::move(cache)), executor_(std::make_shared<tf::Executor>(n))
{
  /** @todo Need to figure out if these can associated with an individual run versus global */
  executor_->make_observer<DebugObserver>("ProcessPlanningObserver");
}

ProcessPlanningServer::ProcessPlanningServer(tesseract::Tesseract::ConstPtr environment, int cache_size, size_t n)
  : cache_(std::make_shared<ProcessEnvironmentCache>(environment, cache_size))
  , executor_(std::make_shared<tf::Executor>(n))
{
  /** @todo Need to figure out if these can associated with an individual run versus global */
  executor_->make_observer<DebugObserver>("ProcessPlanningObserver");
}

void ProcessPlanningServer::registerProcessPlanner(const std::string& name, TaskflowGenerator::UPtr generator)
{
  if (process_planners_.find(name) != process_planners_.end())
    CONSOLE_BRIDGE_logDebug("Process planner %s already exist so replacing with new generator.", name.c_str());

  process_planners_[name] = std::move(generator);
}

void ProcessPlanningServer::loadDefaultProcessPlanners()
{
  registerProcessPlanner(process_planner_names::TRAJOPT_PLANNER_NAME, createTrajOptGenerator());
  registerProcessPlanner(process_planner_names::OMPL_PLANNER_NAME, createOMPLGenerator());
  registerProcessPlanner(process_planner_names::DESCARTES_PLANNER_NAME, createDescartesGenerator());
  registerProcessPlanner(process_planner_names::CARTESIAN_PLANNER_NAME, createCartesianGenerator());
  registerProcessPlanner(process_planner_names::FREESPACE_PLANNER_NAME, createFreespaceGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_PLANNER_NAME, createRasterGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_FT_PLANNER_NAME, createRasterOnlyGenerator());
  registerProcessPlanner(process_planner_names::RASTER_G_FT_PLANNER_NAME, createRasterGlobalGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_DT_PLANNER_NAME, createRasterDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_WAAD_PLANNER_NAME, createRasterWAADGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME, createRasterWAADDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_G_FT_PLANNER_NAME, createRasterOnlyGlobalGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_PLANNER_NAME, createRasterCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_CT_PLANNER_NAME, createRasterOnlyCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_DT_PLANNER_NAME, createRasterCTDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_WAAD_PLANNER_NAME, createRasterCTWAADGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_WAAD_DT_PLANNER_NAME, createRasterCTWAADDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_G_CT_PLANNER_NAME, createRasterGlobalCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_G_CT_PLANNER_NAME, createRasterOnlyGlobalCTGenerator());
}

bool ProcessPlanningServer::hasProcessPlanner(const std::string& name) const
{
  return (process_planners_.find(name) != process_planners_.end());
}

std::vector<std::string> ProcessPlanningServer::getAvailableProcessPlanners() const
{
  std::vector<std::string> planners;
  planners.reserve(process_planners_.size());
  for (const auto& planner : process_planners_)
    planners.push_back(planner.first);

  return planners;
}

ProcessPlanningFuture ProcessPlanningServer::run(const ProcessPlanningRequest& request)
{
  CONSOLE_BRIDGE_logInform("Tesseract Planning Server Recieved Request!");
  ProcessPlanningFuture response;
  response.plan_profile_remapping = std::make_unique<const PlannerProfileRemapping>(request.plan_profile_remapping);
  response.composite_profile_remapping =
      std::make_unique<const PlannerProfileRemapping>(request.composite_profile_remapping);

  response.input = std::make_unique<Instruction>(request.instructions);
  const auto* composite_program = response.input->cast_const<CompositeInstruction>();
  ManipulatorInfo mi = composite_program->getManipulatorInfo();
  response.global_manip_info = std::make_unique<const ManipulatorInfo>(mi);

  bool has_seed{ false };
  if (!isNullInstruction(request.seed))
  {
    has_seed = true;
    response.results = std::make_unique<Instruction>(request.seed);
  }
  else
  {
    response.results = std::make_unique<Instruction>(generateSkeletonSeed(*composite_program));
  }

  auto it = process_planners_.find(request.name);
  if (it == process_planners_.end())
  {
    CONSOLE_BRIDGE_logError("Requested motion Process Pipeline (aka. Taskflow) is not supported!");
    return response;
  }

  tesseract::Tesseract::Ptr tc = cache_->getCachedEnvironment();

  // Set the env state if provided
  if (request.env_state != nullptr)
    tc->getEnvironment()->setState(request.env_state->joints);

  if (!request.commands.empty() && !tc->getEnvironment()->applyCommands(request.commands))
  {
    CONSOLE_BRIDGE_logInform("Tesseract Planning Server Finished Request!");
    return response;
  }

  ProcessInput process_input(tc,
                             response.input.get(),
                             *(response.global_manip_info),
                             *(response.plan_profile_remapping),
                             *(response.composite_profile_remapping),
                             response.results.get(),
                             has_seed,
                             profiles_);
  response.interface = process_input.getProcessInterface();
  response.taskflow_container = it->second->generateTaskflow(process_input, nullptr, nullptr);

  // Dump taskflow graph before running
  if (console_bridge::getLogLevel() >= console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + request.name + "-" + tesseract_common::getTimestampString() +
                  ".dot");
    response.taskflow_container.taskflow->dump(out_data);
    out_data.close();
  }

  response.process_future = executor_->run(*(response.taskflow_container.taskflow));
  return response;
}

std::future<void> ProcessPlanningServer::run(tf::Taskflow& taskflow) { return executor_->run(taskflow); }

void ProcessPlanningServer::waitForAll() { executor_->wait_for_all(); }

void ProcessPlanningServer::enableTaskflowProfiling()
{
  if (profile_observer_ == nullptr)
    profile_observer_ = executor_->make_observer<tf::TFProfObserver>();
}

void ProcessPlanningServer::disableTaskflowProfiling()
{
  if (profile_observer_ != nullptr)
  {
    executor_->remove_observer(profile_observer_);
    profile_observer_ = nullptr;
  }
}

ProfileDictionary::Ptr ProcessPlanningServer::getProfiles() { return profiles_; }

ProfileDictionary::ConstPtr ProcessPlanningServer::getProfiles() const { return profiles_; }

}  // namespace tesseract_planning
