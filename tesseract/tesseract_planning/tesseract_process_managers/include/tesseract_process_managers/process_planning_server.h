/**
 * @file process_planning_server.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <string>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/null_instruction.h>

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_dt_taskflow.h>

namespace tesseract_planning
{
namespace process_planner_names
{
/** @brief TrajOpt Planner */
static const std::string TRAJOPT_PLANNER_NAME = "TrajOptPlanner";

/** @brief OMPL Planner */
static const std::string OMPL_PLANNER_NAME = "OMPLPlanner";

/** @brief Descartes Planner */
static const std::string DESCARTES_PLANNER_NAME = "DescartesPlanner";

/** @brief Freespace Planner */
static const std::string FREESPACE_PLANNER_NAME = "FreespacePlanner";

/** @brief Cartesian Planner */
static const std::string CARTESIAN_PLANNER_NAME = "CartesianPlanner";

/** @brief Raster planner using freespace planner for transitions */
static const std::string RASTER_FT_PLANNER_NAME = "RasterFTPlanner";

/** @brief Raster planner using freespace planner for transitions providing dual transitions */
static const std::string RASTER_FT_DT_PLANNER_NAME = "RasterFTDTPlanner";

/** @brief Raster planner with approach and departure using freespace planner for transitions */
static const std::string RASTER_FT_WAAD_PLANNER_NAME = "RasterFTWAADPlanner";

/** @brief Raster planner with approach and departure using freespace planner for transitions providing dual transitions
 */
static const std::string RASTER_FT_WAAD_DT_PLANNER_NAME = "RasterFTWAADDTPlanner";

/** @brief Raster planner using cartesian planner for transitions */
static const std::string RASTER_CT_PLANNER_NAME = "RasterCTPlanner";

/** @brief Raster planner using cartesian planner for transitions providing dual transitions */
static const std::string RASTER_CT_DT_PLANNER_NAME = "RasterCTDTPlanner";

/** @brief Raster planner with approach and departure using cartesian planner for transitions */
static const std::string RASTER_CT_WAAD_PLANNER_NAME = "RasterCTWAADPlanner";

/** @brief Raster planner with approach and departure using cartesian planner for transitions providing dual transitions
 */
static const std::string RASTER_CT_WAAD_DT_PLANNER_NAME = "RasterCTWAADDTPlanner";

/** @brief Raster planner performs global plan first then macro planning using freespace planner for transitions */
static const std::string RASTER_G_FT_PLANNER_NAME = "RasterGFTPlanner";

/** @brief Raster planner performs global plan first then macro planning using cartesian planner for transitions */
static const std::string RASTER_G_CT_PLANNER_NAME = "RasterGCTPlanner";

/** @brief Raster only planner using freespace planner for transitions */
static const std::string RASTER_O_FT_PLANNER_NAME = "RasterOFTPlanner";

/** @brief Raster only planner using cartesian planner for transitions */
static const std::string RASTER_O_CT_PLANNER_NAME = "RasterOCTPlanner";

/** @brief Raster only planner performs global plan first then macro planning using freespace planner for transitions */
static const std::string RASTER_O_G_FT_PLANNER_NAME = "RasterOGFTPlanner";

/** @brief Raster only planner performs global plan first then macro planning using cartesian planner for transitions */
static const std::string RASTER_O_G_CT_PLANNER_NAME = "RasterOGCTPlanner";
}  // namespace process_planner_names

class TesseractCache
{
public:
  using Ptr = std::shared_ptr<TesseractCache>;
  using ConstPtr = std::shared_ptr<const TesseractCache>;

  TesseractCache(tesseract::Tesseract::Ptr env);

  /**
   * @brief Set the cache size used to hold tesseract objects for motion planning
   * @param size The size of the cache.
   */
  virtual void setCacheSize(long size);

  /**
   * @brief Get the cache size used to hold tesseract objects for motion planning
   * @return The size of the cache.
   */
  virtual long getCacheSize() const;

  /** @brief If the environment has changed it will rebuild the cache of tesseract objects */
  virtual void refreshCache();

  /**
   * @brief This will pop a Tesseract object from the queue
   * @details This will first call refreshCache to ensure it has an updated tesseract then proceed
   */
  virtual tesseract::Tesseract::Ptr getCachedTesseract();

protected:
  /** @brief The tesseract_object used to create the cache */
  tesseract::Tesseract::Ptr tesseract_;

  /** @brief The environment revision number at the time the cache was populated */
  int cache_env_revision_{ 0 };

  /** @brief The assigned cache size */
  std::size_t cache_size_{ 5 };

  /** @brief A vector of cached Tesseact objects */
  std::deque<tesseract::Tesseract::Ptr> cache_;

  /** @brief The mutex used when reading and writing to cache_ */
  mutable std::shared_mutex cache_mutex_;
};

struct ProcessPlanningRequest
{
  /** @brief The name of the planner to use */
  std::string name;

  /** @brief  This should an xml string of the command language instructions */
  Instruction instructions{ NullInstruction() };

  /** @brief This should an xml string of the command language instructions (Optional) */
  Instruction seed{ NullInstruction() };

  /** @brief Environment state to start planning with (Optional)  */
  tesseract_environment::EnvState::ConstPtr env_state;

  /** @brief Additional Commands to be applied to environment prior to planning (Optional) */
  tesseract_environment::Commands commands;

  /** @brief Enable debug content (Optional) */
  bool debug{ false };

  /** @brief Enable profiling of the planning request (Optional) */
  bool profile{ false };

  /**
   * @brief This allows the remapping of the Plan Profile identified in the command language to a specific profile for a
   * given motion planner. (Optional)
   */
  PlannerProfileRemapping plan_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner. (Optional)
   */
  PlannerProfileRemapping composite_profile_remapping;
};

/**
 * @brief This contains the result for the process planning request
 * @details Must chec the status before access the results to know if available.
 * @note This must not go out of scope until the process has finished
 */
struct ProcessPlanningFuture
{
  std::future<void> process_future;
  std::unique_ptr<bool> success{ std::make_unique<bool>(true) };
  std::unique_ptr<Instruction> input;
  std::unique_ptr<Instruction> results;
  std::unique_ptr<const ManipulatorInfo> global_manip_info;
  std::unique_ptr<const PlannerProfileRemapping> plan_profile_remapping;
  std::unique_ptr<const PlannerProfileRemapping> composite_profile_remapping;
  TaskflowGenerator::UPtr taskflow_generator;
  bool isReady() const { return (process_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready); }
};

using ProcessPlannerGeneratorFn = std::function<TaskflowGenerator::UPtr(const ProcessPlanningRequest& request)>;

class ProcessPlanningServer
{
public:
  ProcessPlanningServer(TesseractCache::Ptr cache, size_t n = std::thread::hardware_concurrency());
  virtual ~ProcessPlanningServer() = default;

  bool registerProcessPlanner(const std::string& name, ProcessPlannerGeneratorFn generator);

  ProcessPlanningFuture run(const ProcessPlanningRequest& request);

  std::future<void> run(tf::Taskflow& taskflow);

  void waitForAll();

  /** @brief Get and modify profiles */
  TrajOptCompositeProfileMap& getTrajOptCompositeProfiles();
  const TrajOptCompositeProfileMap& getTrajOptCompositeProfiles() const;

  TrajOptPlanProfileMap& getTrajOptPlanProfiles();
  const TrajOptPlanProfileMap& getTrajOptPlanProfiles() const;

  DescartesPlanProfileMap<double>& getDescartesPlanProfiles();
  const DescartesPlanProfileMap<double>& getDescartesPlanProfiles() const;

  OMPLPlanProfileMap& getOMPLPlanProfiles();
  const OMPLPlanProfileMap& getOMPLPlanProfiles() const;

  SimplePlannerCompositeProfileMap& getSimplePlannerCompositeProfiles();
  const SimplePlannerCompositeProfileMap& getSimplePlannerCompositeProfiles() const;

  SimplePlannerPlanProfileMap& getSimplePlannerPlanProfiles();
  const SimplePlannerPlanProfileMap& getSimplePlannerPlanProfiles() const;

protected:
  TesseractCache::Ptr cache_;
  std::shared_ptr<tf::Executor> executor_;

  std::unordered_map<std::string, ProcessPlannerGeneratorFn> process_planners_;

  /** @brief Trajopt available composite profiles */
  TrajOptCompositeProfileMap trajopt_composite_profiles_;

  /**@brief The trajopt available plan profiles */
  TrajOptPlanProfileMap trajopt_plan_profiles_;

  /** @brief The Descartes available plan profiles */
  DescartesPlanProfileMap<double> descartes_plan_profiles_;

  /** @brief The OMPL available plan profiles */
  OMPLPlanProfileMap ompl_plan_profiles_;

  /** @brief The Simple Planner available plan profiles */
  SimplePlannerPlanProfileMap simple_plan_profiles_;

  /** @brief The Simple Planner available composite profiles */
  SimplePlannerCompositeProfileMap simple_composite_profiles_;

  RasterTaskflow::UPtr createRasterTaskflow(const ProcessPlanningRequest& request);
  RasterOnlyTaskflow::UPtr createRasterOnlyTaskflow(const ProcessPlanningRequest& request);
  RasterGlobalTaskflow::UPtr createRasterGlobalTaskflow(const ProcessPlanningRequest& request);
  RasterDTTaskflow::UPtr createRasterDTTaskflow(const ProcessPlanningRequest& request);
  RasterWAADTaskflow::UPtr createRasterWAADTaskflow(const ProcessPlanningRequest& request);
  RasterWAADDTTaskflow::UPtr createRasterWAADDTTaskflow(const ProcessPlanningRequest& request);
  RasterOnlyGlobalTaskflow::UPtr createRasterOnlyGlobalTaskflow(const ProcessPlanningRequest& request);
  RasterTaskflow::UPtr createRasterCTTaskflow(const ProcessPlanningRequest& request);
  RasterOnlyTaskflow::UPtr createRasterOnlyCTTaskflow(const ProcessPlanningRequest& request);
  RasterDTTaskflow::UPtr createRasterCTDTTaskflow(const ProcessPlanningRequest& request);
  RasterWAADTaskflow::UPtr createRasterCTWAADTaskflow(const ProcessPlanningRequest& request);
  RasterWAADDTTaskflow::UPtr createRasterCTWAADDTTaskflow(const ProcessPlanningRequest& request);
  RasterGlobalTaskflow::UPtr createRasterGlobalCTTaskflow(const ProcessPlanningRequest& request);
  RasterOnlyGlobalTaskflow::UPtr createRasterOnlyGlobalCTTaskflow(const ProcessPlanningRequest& request);
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
