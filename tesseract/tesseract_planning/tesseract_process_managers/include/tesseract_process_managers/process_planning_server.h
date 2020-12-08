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

#include <tesseract_process_managers/process_environment_cache.h>
#include <tesseract_process_managers/taskflow_generator.h>
#include <tesseract_process_managers/process_planning_request.h>
#include <tesseract_process_managers/process_planning_future.h>

namespace tesseract_planning
{
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

  void registerProcessPlanner(const std::string& name, TaskflowGenerator::UPtr generator);
  void loadDefaultProcessPlanners();

  bool hasProcessPlanner(const std::string& name) const;
  std::vector<std::string> getAvailableProcessPlanners() const;

  ProcessPlanningFuture run(const ProcessPlanningRequest& request);

  std::future<void> run(tf::Taskflow& taskflow);

  void waitForAll();

  ProfileDictionary::Ptr getProfiles();

  ProfileDictionary::ConstPtr getProfiles() const;

protected:
  EnvironmentCache::Ptr cache_;
  std::shared_ptr<tf::Executor> executor_;

  std::unordered_map<std::string, TaskflowGenerator::UPtr> process_planners_;
  ProfileDictionary::Ptr profiles_{ std::make_shared<ProfileDictionary>() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
