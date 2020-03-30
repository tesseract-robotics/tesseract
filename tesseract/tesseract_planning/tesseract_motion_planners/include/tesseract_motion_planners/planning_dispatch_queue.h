/**
 * @file planning_dispatch_queue.h
 * @brief A planning async dispatch queue to run plans in parallel.
 *
 * This builds on the code provided in the link below:
 * https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/dispatch.cpp
 *
 * @author Levi Armstrong
 * @date February 24, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNING_DISPATCH_QUEUE_H
#define TESSERACT_MOTION_PLANNERS_PLANNING_DISPATCH_QUEUE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <thread>
#include <future>
#include <functional>
#include <vector>
#include <cstdint>
#include <cstdio>
#include <queue>
#include <deque>
#include <mutex>
#include <string>
#include <condition_variable>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/types.h>
namespace tesseract_motion_planners
{
class PlanningDispatchQueue
{
public:
  using WorkerFunc = std::function<void(tesseract_motion_planners::PlannerResponse&)>;
  using TaskID = std::size_t;
  using TaskResult = std::pair<TaskID, tesseract_motion_planners::PlannerResponse>;

  PlanningDispatchQueue(std::string name, std::size_t thread_cnt = 1);
  ~PlanningDispatchQueue();
  PlanningDispatchQueue(const PlanningDispatchQueue& rhs) = delete;
  PlanningDispatchQueue& operator=(const PlanningDispatchQueue& rhs) = delete;
  PlanningDispatchQueue(PlanningDispatchQueue&& rhs) = delete;
  PlanningDispatchQueue& operator=(PlanningDispatchQueue&& rhs) = delete;

  // dispatch and copy
  TaskID dispatchPlanningRequest(const WorkerFunc& fn);
  // dispatch and move
  TaskID dispatchPlanningRequest(WorkerFunc&& fn);

  bool hasResults();
  bool fetchResult(TaskResult& result);
  bool fetchResults(std::vector<TaskResult>& results);
  bool isIdle();

private:
  using TaskRequest = std::pair<TaskID, WorkerFunc>;

  std::string name_;
  std::mutex lock_;
  std::vector<std::thread> threads_;
  std::queue<TaskRequest> func_queue_;
  std::deque<TaskResult> results_queue_;
  std::condition_variable cv_;
  bool quit_ = false;
  std::size_t id{ 0 };

  void dispatch_thread_handler();
};

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_PLANNING_DISPATCH_QUEUE_H
