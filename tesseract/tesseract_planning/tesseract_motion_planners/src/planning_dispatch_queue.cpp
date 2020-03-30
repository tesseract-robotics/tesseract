/**
 * @file planning_dispatch_queue.cpp
 * @brief A planning async dispatch queue to run plans in parallel.
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

#include <tesseract_motion_planners/planning_dispatch_queue.h>

namespace tesseract_motion_planners
{
PlanningDispatchQueue::PlanningDispatchQueue(std::string name, std::size_t thread_cnt)
  : name_{ std::move(name) }, threads_(thread_cnt)
{
  for (size_t i = 0; i < threads_.size(); i++)
  {
    threads_[i] = std::thread(&PlanningDispatchQueue::dispatch_thread_handler, this);
  }
}

PlanningDispatchQueue::~PlanningDispatchQueue()
{
  // Notify all threads that it's time to wrap up
  std::unique_lock<std::mutex> lock(lock_);
  quit_ = true;
  lock.unlock();
  cv_.notify_all();

  // Wait for threads to finish before we exit
  for (size_t i = 0; i < threads_.size(); i++)
  {
    if (threads_[i].joinable())
    {
      printf("Destructor: Joining thread %zu until completion\n", i);
      threads_[i].join();
    }
  }
}

PlanningDispatchQueue::TaskID PlanningDispatchQueue::dispatchPlanningRequest(const WorkerFunc& fn)
{
  std::unique_lock<std::mutex> lock(lock_);
  TaskID task_id = ++id;
  func_queue_.push(std::make_pair(task_id, fn));

  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again (see notify_one for details)
  lock.unlock();
  cv_.notify_all();

  return task_id;
}

PlanningDispatchQueue::TaskID PlanningDispatchQueue::dispatchPlanningRequest(WorkerFunc&& fn)
{
  std::unique_lock<std::mutex> lock(lock_);
  TaskID task_id = ++id;
  func_queue_.push(std::make_pair(task_id, std::move(fn)));

  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again (see notify_one for details)
  lock.unlock();
  cv_.notify_all();

  return task_id;
}

void PlanningDispatchQueue::dispatch_thread_handler()
{
  std::unique_lock<std::mutex> lock(lock_);

  do
  {
    // Wait until we have data or a quit signal
    cv_.wait(lock, [this] { return (func_queue_.size() || quit_); });

    // after wait, we own the lock
    if (!quit_ && func_queue_.size())
    {
      auto fn = std::move(func_queue_.front());
      func_queue_.pop();

      // unlock now that we're done messing with the queue
      lock.unlock();
      tesseract_motion_planners::PlannerResponse response;
      fn.second(response);

      lock.lock();
      results_queue_.push_back(std::make_pair(fn.first, response));
    }
  } while (!quit_);
}

bool PlanningDispatchQueue::hasResults()
{
  std::unique_lock<std::mutex> lock(lock_);
  bool has_results = !results_queue_.empty();
  lock.unlock();
  return has_results;
}

bool PlanningDispatchQueue::fetchResult(TaskResult& result)
{
  std::unique_lock<std::mutex> lock(lock_);

  if (results_queue_.empty())
  {
    lock.unlock();
    return false;
  }

  result = std::move(results_queue_.front());
  results_queue_.pop_front();
  lock.unlock();
  return true;
}

bool PlanningDispatchQueue::fetchResults(std::vector<TaskResult>& results)
{
  std::unique_lock<std::mutex> lock(lock_);

  if (results_queue_.empty())
  {
    lock.unlock();
    return false;
  }

  std::move(results_queue_.begin(), results_queue_.end(), std::back_inserter(results));
  results_queue_.erase(results_queue_.begin(), results_queue_.end());
  lock.unlock();
  return true;
}

bool PlanningDispatchQueue::isIdle()
{
  std::unique_lock<std::mutex> lock(lock_);
  bool is_idle = func_queue_.empty();
  lock.unlock();
  return is_idle;
}

}  // namespace tesseract_motion_planners
