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

#include <tesseract_motion_planners/core/profile_dictionary.h>

#include <tesseract_process_managers/core/process_environment_cache.h>
#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::ProcessPlanningServer)
%nodefaultctor tesseract_planning::ProcessPlanningServer;
#endif  // SWIG

namespace tesseract_planning
{
/**
 * @brief A process planning server that support asynchronous exectuion of process planning requests
 * @details It allows the developer to register Process pipelines (aka. Taskflow Generators) so they may be request
 */
class ProcessPlanningServer
{
public:
  using Ptr = std::shared_ptr<ProcessPlanningServer>;
  using ConstPtr = std::shared_ptr<const ProcessPlanningServer>;

#ifndef SWIG
  /**
   * @brief Constructor
   * @param cache The cache to use for getting Environment objects
   * @param n The number of threads used by the planning server
   */
  ProcessPlanningServer(EnvironmentCache::Ptr cache, size_t n = std::thread::hardware_concurrency());
#endif  // SWIG

  /**
   * @brief Constructor
   * @param environment The environment object to leverage
   * @param cache_size The cache size used for maintaining a que of environments for improved performance when making
   * multiple requests
   * @param n The number of threads used by the planning server
   */
  ProcessPlanningServer(tesseract_environment::Environment::ConstPtr environment,
                        int cache_size = 1,
                        size_t n = std::thread::hardware_concurrency());

  virtual ~ProcessPlanningServer() = default;
#ifndef SWIG
  ProcessPlanningServer(const ProcessPlanningServer&) = default;
  ProcessPlanningServer& operator=(const ProcessPlanningServer&) = default;
  ProcessPlanningServer(ProcessPlanningServer&&) = default;
  ProcessPlanningServer& operator=(ProcessPlanningServer&&) = default;
#endif  // SWIG

#ifndef SWIG
  /**
   * @brief Register a process planner with the planning server
   * @param name The name used to locate the process planner through requests
   * @param generator The Taskflow Generator associated with the name
   */
  void registerProcessPlanner(const std::string& name, TaskflowGenerator::UPtr generator);
#endif  // SWIG

  /**
   * @brief Load default process planners
   * @details This is not called automatically, so user but call this to load default planners.
   */
  void loadDefaultProcessPlanners();

  /**
   * @brief Check if the planning server has a name associated with a process pipeline
   * @param name The name of the process planner to check for
   * @return True if the name is already taken, otherwise false
   */
  bool hasProcessPlanner(const std::string& name) const;

  /**
   * @brief Get a list of process planner registered with the planning server
   * @return A vector of names
   */
  std::vector<std::string> getAvailableProcessPlanners() const;

#ifndef SWIG
  /**
   * @brief Execute a process planning request.
   * @details This does not block to allow for multiple requests, use future to wait if needed.
   * @param request The process planning request to execute
   * @return A process planning future to get results and monitor the execution along with the ability to abort
   */
  ProcessPlanningFuture run(const ProcessPlanningRequest& request);

  /**
   * @brief This is a utility function to run arbitrary taskflows
   * @param taskflow the taskflow to execute
   * @return A future to monitor progress
   */
  std::future<void> run(tf::Taskflow& taskflow);
#endif  // SWIG

#ifdef SWIG
  %extend
  {
    std::shared_ptr<tesseract_planning::ProcessPlanningFuture> run(const ProcessPlanningRequest& request)
    {
      return std::make_shared<tesseract_planning::ProcessPlanningFuture>(std::move($self->run(request)));
    }
  }
#endif  // SWIG

  /** @brief Wait for all process currently being executed to finish before returning */
  void waitForAll();

  /** @brief This add a Taskflow profiling observer to the executor */
  void enableTaskflowProfiling();

  /** @brief This remove the Taskflow profiling observer from the executor if one exists */
  void disableTaskflowProfiling();

  /**
   * @brief Get the profile dictionary associated with the planning server
   * @return Profile dictionary
   */
  ProfileDictionary::Ptr getProfiles();

  /**
   * @brief Get the profile dictionary associated with the planning server (const)
   * @return Profile dictionary (const)
   */
  ProfileDictionary::ConstPtr getProfiles() const;

protected:
  EnvironmentCache::Ptr cache_;
  std::shared_ptr<tf::Executor> executor_;
  std::shared_ptr<tf::TFProfObserver> profile_observer_;

  std::unordered_map<std::string, TaskflowGenerator::UPtr> process_planners_;
  ProfileDictionary::Ptr profiles_{ std::make_shared<ProfileDictionary>() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
