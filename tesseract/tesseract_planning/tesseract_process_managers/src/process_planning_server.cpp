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

#include <tesseract_process_managers/process_planning_server.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflows/descartes_taskflow.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>
#include <tesseract_process_managers/taskflows/ompl_taskflow.h>
#include <tesseract_process_managers/taskflows/trajopt_taskflow.h>

#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
TesseractCache::TesseractCache(tesseract::Tesseract::Ptr env) : tesseract_(std::move(env)) {}
void TesseractCache::setCacheSize(long size)
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  cache_size_ = static_cast<std::size_t>(size);
}

long TesseractCache::getCacheSize() const { return static_cast<long>(cache_size_); }

void TesseractCache::refreshCache()
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract::Tesseract::Ptr thor;

  int rev = tesseract_->getEnvironment()->getRevision();
  if (rev != cache_env_revision_ || cache_.empty())
  {
    thor = tesseract_->clone();
    cache_env_revision_ = rev;
  }

  if (thor != nullptr)
  {
    cache_.clear();
    for (std::size_t i = 0; i < cache_size_; ++i)
      cache_.push_back(thor->clone());
  }
  else if (cache_.size() <= 2)
  {
    for (std::size_t i = (cache_.size() - 1); i < cache_size_; ++i)
      cache_.push_back(cache_.front()->clone());
  }
}

tesseract::Tesseract::Ptr TesseractCache::getCachedTesseract()
{
  // This is to make sure the cached items are updated if needed
  refreshCache();

  tesseract_environment::EnvState current_state;
  current_state = *(tesseract_->getEnvironment()->getCurrentState());

  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract::Tesseract::Ptr t = cache_.back();

  // Update to the current joint values
  t->getEnvironment()->setState(current_state.joints);

  cache_.pop_back();

  return t;
}

ProcessPlanningServer::ProcessPlanningServer(TesseractCache::Ptr cache, size_t n)
  : cache_(std::move(cache)), executor_(std::make_shared<tf::Executor>(n))
{
}

bool ProcessPlanningServer::registerProcessPlanner(const std::string& name, ProcessPlannerGeneratorFn generator)
{
  if (process_planners_.find(name) != process_planners_.end())
    CONSOLE_BRIDGE_logDebug("Process planner %s already exist so replacing with new generator.", name);

  process_planners_[name] = generator;
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

  if (!isNullInstruction(request.seed))
    response.results = std::make_unique<Instruction>(request.seed);
  else
    response.results = std::make_unique<Instruction>(generateSkeletonSeed(*composite_program));

  if (request.name == process_planner_names::TRAJOPT_PLANNER_NAME)
  {
    TrajOptTaskflowParams params;
    params.enable_simple_planner = isNullInstruction(request.seed);
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    response.taskflow_generator = createTrajOptTaskflow(params);
  }
  else if (request.name == process_planner_names::OMPL_PLANNER_NAME)
  {
    OMPLTaskflowParams params;
    params.enable_simple_planner = isNullInstruction(request.seed);
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.ompl_plan_profiles = ompl_plan_profiles_;
    response.taskflow_generator = createOMPLTaskflow(params);
  }
  else if (request.name == process_planner_names::DESCARTES_PLANNER_NAME)
  {
    DescartesTaskflowParams params;
    params.enable_simple_planner = isNullInstruction(request.seed);
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    response.taskflow_generator = createDescartesTaskflow(params);
  }
  else if (request.name == process_planner_names::CARTESIAN_PLANNER_NAME)
  {
    CartesianTaskflowParams params;
    params.enable_simple_planner = isNullInstruction(request.seed);
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    response.taskflow_generator = createCartesianTaskflow(params);
  }
  else if (request.name == process_planner_names::FREESPACE_PLANNER_NAME)
  {
    FreespaceTaskflowParams params;
    params.enable_simple_planner = isNullInstruction(request.seed);
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.ompl_plan_profiles = ompl_plan_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    response.taskflow_generator = createFreespaceTaskflow(params);
  }
  else if (request.name == process_planner_names::RASTER_FT_PLANNER_NAME)
    response.taskflow_generator = createRasterTaskflow(request);
  else if (request.name == process_planner_names::RASTER_O_FT_PLANNER_NAME)
    response.taskflow_generator = createRasterOnlyTaskflow(request);
  else if (request.name == process_planner_names::RASTER_G_FT_PLANNER_NAME)
    response.taskflow_generator = createRasterGlobalTaskflow(request);
  else if (request.name == process_planner_names::RASTER_FT_DT_PLANNER_NAME)
    response.taskflow_generator = createRasterDTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_FT_WAAD_PLANNER_NAME)
    response.taskflow_generator = createRasterWAADTaskflow(request);
  else if (request.name == process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME)
    response.taskflow_generator = createRasterWAADDTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_O_G_FT_PLANNER_NAME)
    response.taskflow_generator = createRasterOnlyGlobalTaskflow(request);
  else if (request.name == process_planner_names::RASTER_CT_PLANNER_NAME)
    response.taskflow_generator = createRasterDTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_O_CT_PLANNER_NAME)
    response.taskflow_generator = createRasterOnlyCTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_CT_DT_PLANNER_NAME)
    response.taskflow_generator = createRasterCTDTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_CT_WAAD_PLANNER_NAME)
    response.taskflow_generator = createRasterCTWAADTaskflow(request);
  else if (request.name == process_planner_names::RASTER_CT_WAAD_DT_PLANNER_NAME)
    response.taskflow_generator = createRasterCTWAADDTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_G_CT_PLANNER_NAME)
    response.taskflow_generator = createRasterGlobalCTTaskflow(request);
  else if (request.name == process_planner_names::RASTER_O_G_CT_PLANNER_NAME)
    response.taskflow_generator = createRasterOnlyGlobalCTTaskflow(request);
  else
  {
    CONSOLE_BRIDGE_logError("Requested motion planner is not supported!");
    return response;
  }

  assert(response.taskflow_generator != nullptr);
  tesseract::Tesseract::Ptr tc = cache_->getCachedTesseract();

  // Set the env state if provided
  if (request.env_state != nullptr)
    tc->getEnvironment()->setState(request.env_state->joints);

  if (!request.commands.empty() && !tc->getEnvironment()->applyCommands(request.commands))
  {
    CONSOLE_BRIDGE_logInform("Tesseract Planning Server Finished Request!");
    return response;
  }

  //  response.process_manager->enableDebug(request.debug);
  //  response.process_manager->enableProfile(request.profile);
  bool* s = response.success.get();
  auto done_cb = [s]() {
    *s &= true;
    CONSOLE_BRIDGE_logError("Done Callback");
  };
  auto error_cb = [s]() {
    *s = false;
    CONSOLE_BRIDGE_logError("Error Callback");
  };

  response.process_future =
      executor_->run(response.taskflow_generator->generateTaskflow(ProcessInput(tc,
                                                                                response.input.get(),
                                                                                *(response.global_manip_info),
                                                                                *(response.plan_profile_remapping),
                                                                                *(response.composite_profile_remapping),
                                                                                response.results.get(),
                                                                                request.debug),
                                                                   done_cb,
                                                                   error_cb));
  return response;
}

std::future<void> ProcessPlanningServer::run(tf::Taskflow& taskflow) { return executor_->run(taskflow); }

void ProcessPlanningServer::waitForAll() { executor_->wait_for_all(); }

TrajOptCompositeProfileMap& ProcessPlanningServer::getTrajOptCompositeProfiles() { return trajopt_composite_profiles_; }

const TrajOptCompositeProfileMap& ProcessPlanningServer::getTrajOptCompositeProfiles() const
{
  return trajopt_composite_profiles_;
}

TrajOptPlanProfileMap& ProcessPlanningServer::getTrajOptPlanProfiles() { return trajopt_plan_profiles_; }

const TrajOptPlanProfileMap& ProcessPlanningServer::getTrajOptPlanProfiles() const { return trajopt_plan_profiles_; }

DescartesPlanProfileMap<double>& ProcessPlanningServer::getDescartesPlanProfiles() { return descartes_plan_profiles_; }

const DescartesPlanProfileMap<double>& ProcessPlanningServer::getDescartesPlanProfiles() const
{
  return descartes_plan_profiles_;
}

OMPLPlanProfileMap& ProcessPlanningServer::getOMPLPlanProfiles() { return ompl_plan_profiles_; }

const OMPLPlanProfileMap& ProcessPlanningServer::getOMPLPlanProfiles() const { return ompl_plan_profiles_; }

SimplePlannerCompositeProfileMap& ProcessPlanningServer::getSimplePlannerCompositeProfiles()
{
  return simple_composite_profiles_;
}

const SimplePlannerCompositeProfileMap& ProcessPlanningServer::getSimplePlannerCompositeProfiles() const
{
  return simple_composite_profiles_;
}

SimplePlannerPlanProfileMap& ProcessPlanningServer::getSimplePlannerPlanProfiles() { return simple_plan_profiles_; }

const SimplePlannerPlanProfileMap& ProcessPlanningServer::getSimplePlannerPlanProfiles() const
{
  return simple_plan_profiles_;
}

RasterTaskflow::UPtr ProcessPlanningServer::createRasterTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = isNullInstruction(request.seed);
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(fparams);
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = isNullInstruction(request.seed);
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cparams);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterOnlyTaskflow::UPtr ProcessPlanningServer::createRasterOnlyTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams tparams;
  tparams.enable_simple_planner = isNullInstruction(request.seed);
  tparams.simple_plan_profiles = simple_plan_profiles_;
  tparams.simple_composite_profiles = simple_composite_profiles_;
  tparams.ompl_plan_profiles = ompl_plan_profiles_;
  tparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  tparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(tparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = isNullInstruction(request.seed);
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cparams);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

RasterGlobalTaskflow::UPtr ProcessPlanningServer::createRasterGlobalTaskflow(const ProcessPlanningRequest& request)
{
  DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = isNullInstruction(request.seed);
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  GraphTaskflow::UPtr global_task = createDescartesTaskflow(global_params);

  FreespaceTaskflowParams freespace_params;
  freespace_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  freespace_params.enable_simple_planner = false;
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(freespace_params);

  TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createTrajOptTaskflow(raster_params);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterDTTaskflow::UPtr ProcessPlanningServer::createRasterDTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = isNullInstruction(request.seed);
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(fparams);
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = isNullInstruction(request.seed);
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cparams);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterWAADTaskflow::UPtr ProcessPlanningServer::createRasterWAADTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = isNullInstruction(request.seed);
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(fparams);
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = isNullInstruction(request.seed);
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cparams);

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterWAADDTTaskflow::UPtr ProcessPlanningServer::createRasterWAADDTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = isNullInstruction(request.seed);
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(fparams);
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = isNullInstruction(request.seed);
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cparams);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterOnlyGlobalTaskflow::UPtr
ProcessPlanningServer::createRasterOnlyGlobalTaskflow(const ProcessPlanningRequest& request)
{
  DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = isNullInstruction(request.seed);
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  GraphTaskflow::UPtr global_task = createDescartesTaskflow(global_params);

  FreespaceTaskflowParams transition_params;
  transition_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  transition_params.enable_simple_planner = false;
  transition_params.simple_plan_profiles = simple_plan_profiles_;
  transition_params.simple_composite_profiles = simple_composite_profiles_;
  transition_params.ompl_plan_profiles = ompl_plan_profiles_;
  transition_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  transition_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr transition_task = createFreespaceTaskflow(transition_params);

  TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createTrajOptTaskflow(raster_params);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}

RasterTaskflow::UPtr ProcessPlanningServer::createRasterCTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = isNullInstruction(request.seed);
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = isNullInstruction(request.seed);
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cartesian_params);
  GraphTaskflow::UPtr transition_task = createCartesianTaskflow(cartesian_params);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterOnlyTaskflow::UPtr ProcessPlanningServer::createRasterOnlyCTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Transition and Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = isNullInstruction(request.seed);
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cartesian_params);
  GraphTaskflow::UPtr transition_task = createCartesianTaskflow(cartesian_params);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

RasterDTTaskflow::UPtr ProcessPlanningServer::createRasterCTDTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = isNullInstruction(request.seed);
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = isNullInstruction(request.seed);
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cartesian_params);
  GraphTaskflow::UPtr transition_task = createCartesianTaskflow(cartesian_params);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterWAADTaskflow::UPtr ProcessPlanningServer::createRasterCTWAADTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = isNullInstruction(request.seed);
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = isNullInstruction(request.seed);
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cartesian_params);
  GraphTaskflow::UPtr transition_task = createCartesianTaskflow(cartesian_params);

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterWAADDTTaskflow::UPtr ProcessPlanningServer::createRasterCTWAADDTTaskflow(const ProcessPlanningRequest& request)
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = isNullInstruction(request.seed);
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = isNullInstruction(request.seed);
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createCartesianTaskflow(cartesian_params);
  GraphTaskflow::UPtr transition_task = createCartesianTaskflow(cartesian_params);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterGlobalTaskflow::UPtr ProcessPlanningServer::createRasterGlobalCTTaskflow(const ProcessPlanningRequest& request)
{
  DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = isNullInstruction(request.seed);
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  GraphTaskflow::UPtr global_task = createDescartesTaskflow(global_params);

  FreespaceTaskflowParams freespace_params;
  freespace_params.type = FreespaceTaskflowType::TRAJOPT_FIRST;
  freespace_params.enable_simple_planner = false;
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr freespace_task = createFreespaceTaskflow(freespace_params);

  TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createTrajOptTaskflow(raster_params);
  GraphTaskflow::UPtr transition_task = createTrajOptTaskflow(raster_params);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

RasterOnlyGlobalTaskflow::UPtr
ProcessPlanningServer::createRasterOnlyGlobalCTTaskflow(const ProcessPlanningRequest& request)
{
  DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = isNullInstruction(request.seed);
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  GraphTaskflow::UPtr global_task = createDescartesTaskflow(global_params);

  TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  GraphTaskflow::UPtr raster_task = createTrajOptTaskflow(raster_params);
  GraphTaskflow::UPtr transition_task = createTrajOptTaskflow(raster_params);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}
}  // namespace tesseract_planning
