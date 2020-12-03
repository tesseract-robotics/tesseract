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
#include <string>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/null_instruction.h>

#include <tesseract_motion_planners/core/profile_dictionary.h>
#include <tesseract_process_managers/process_environment_cache.h>
#include <tesseract_process_managers/taskflow_generator.h>

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

/** @brief Raster planner with approach and departure using freespace planner for transitions with dual transitions */
static const std::string RASTER_FT_WAAD_DT_PLANNER_NAME = "RasterFTWAADDTPlanner";

/** @brief Raster planner using cartesian planner for transitions */
static const std::string RASTER_CT_PLANNER_NAME = "RasterCTPlanner";

/** @brief Raster planner using cartesian planner for transitions providing dual transitions */
static const std::string RASTER_CT_DT_PLANNER_NAME = "RasterCTDTPlanner";

/** @brief Raster planner with approach and departure using cartesian planner for transitions */
static const std::string RASTER_CT_WAAD_PLANNER_NAME = "RasterCTWAADPlanner";

/** @brief Raster planner with approach and departure using cartesian planner for transitions wit dual transitions */
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

  /** @brief Clear all content */
  void clear()
  {
    *success = true;
    input = nullptr;
    results = nullptr;
    global_manip_info = nullptr;
    plan_profile_remapping = nullptr;
    composite_profile_remapping = nullptr;
    taskflow_generator->clear();
    taskflow_generator = nullptr;
  }

  // These methods are used for checking the status of the future
  bool ready() const { return (process_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready); }
  void wait() const { process_future.wait(); }

  template <typename R, typename P>
  std::future_status waitFor(const std::chrono::duration<R, P>& duration) const
  {
    return process_future.wait_for(duration);
  }

  template <typename C, typename D>
  std::future_status waitUntil(const std::chrono::time_point<C, D>& abs) const
  {
    return process_future.wait_until(abs);
  }
};

using ProcessPlannerGeneratorFn =
    std::function<TaskflowGenerator::UPtr(const ProcessPlanningRequest& request, ProfileDictionary::ConstPtr profiles)>;

class ProcessPlanningServer
{
public:
  using Ptr = std::shared_ptr<ProcessPlanningServer>;
  using ConstPtr = std::shared_ptr<const ProcessPlanningServer>;

  ProcessPlanningServer(EnvironmentCache::Ptr cache, size_t n = std::thread::hardware_concurrency());
  virtual ~ProcessPlanningServer() = default;
  ProcessPlanningServer(const ProcessPlanningServer&) = default;
  ProcessPlanningServer& operator=(const ProcessPlanningServer&) = default;
  ProcessPlanningServer(ProcessPlanningServer&&) = default;
  ProcessPlanningServer& operator=(ProcessPlanningServer&&) = default;

  void registerProcessPlanner(const std::string& name, ProcessPlannerGeneratorFn generator);
  void loadDefaultProcessPlanners();

  bool hasProcessPlanner(const std::string& name) const;

  ProcessPlanningFuture run(const ProcessPlanningRequest& request);

  std::future<void> run(tf::Taskflow& taskflow);

  void waitForAll();

  ProfileDictionary::Ptr getProfiles();

  ProfileDictionary::ConstPtr getProfiles() const;

protected:
  EnvironmentCache::Ptr cache_;
  std::shared_ptr<tf::Executor> executor_;

  std::unordered_map<std::string, ProcessPlannerGeneratorFn> process_planners_;
  ProfileDictionary::Ptr profiles_{ std::make_shared<ProfileDictionary>() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
