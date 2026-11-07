/**
 * @file per_thread_contact_manager.h
 * @brief A contact manager that gives each calling thread its own clone of a prototype
 *
 * @author Jelle Feringa
 * @date November 7, 2026
 *
 * @copyright Copyright (c) 2026, Jelle Feringa
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
#ifndef TESSERACT_COLLISION_PER_THREAD_CONTACT_MANAGER_H
#define TESSERACT_COLLISION_PER_THREAD_CONTACT_MANAGER_H

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

namespace tesseract::collision
{
/**
 * @brief One contact manager per calling thread, all cloned from one configured prototype.
 *
 * A contact manager is not thread safe: a contact test reads the transforms the last setCollisionObjectsTransform
 * wrote. Code that may be called from several threads at once (a motion planner's collision evaluators, state
 * validators, edge evaluators) holds one of these instead of a manager, and calls get() to obtain the calling
 * thread's own manager.
 *
 * - The prototype is configured once, before construction, and is never handed out or used for a contact test.
 * - A thread's manager is cloned from the prototype on that thread's first call to get(), keyed by std::thread::id.
 * - Managers live until this object is destroyed. A thread that is given the id of a thread that has exited reuses
 *   that thread's manager, which is safe because no two live threads share an id.
 * - A thread's manager keeps the state that thread left in it, such as object transforms. Callers set every
 *   transform they rely on before each contact test.
 *
 * @tparam ManagerT DiscreteContactManager or ContinuousContactManager
 */
template <typename ManagerT>
class PerThreadContactManager
{
public:
  using Ptr = std::shared_ptr<PerThreadContactManager<ManagerT>>;
  using ConstPtr = std::shared_ptr<const PerThreadContactManager<ManagerT>>;
  using UPtr = std::unique_ptr<PerThreadContactManager<ManagerT>>;
  using ConstUPtr = std::unique_ptr<const PerThreadContactManager<ManagerT>>;

  /**
   * @brief Take ownership of a configured prototype
   * @param prototype The manager every thread's manager is cloned from
   * @throws std::invalid_argument if prototype is null
   */
  explicit PerThreadContactManager(std::unique_ptr<const ManagerT> prototype) : prototype_(std::move(prototype))
  {
    if (prototype_ == nullptr)
      throw std::invalid_argument("PerThreadContactManager, prototype contact manager is null");
  }

  ~PerThreadContactManager() = default;
  PerThreadContactManager(const PerThreadContactManager&) = delete;
  PerThreadContactManager& operator=(const PerThreadContactManager&) = delete;
  PerThreadContactManager(PerThreadContactManager&&) = delete;
  PerThreadContactManager& operator=(PerThreadContactManager&&) = delete;

  /**
   * @brief The calling thread's manager, cloned from the prototype on this thread's first call
   * @details Thread safe. The returned manager is only ever returned to the calling thread.
   */
  ManagerT& get() const
  {
    const std::thread::id id = std::this_thread::get_id();

    {
      const std::shared_lock<std::shared_mutex> lock(mutex_);
      auto it = managers_.find(id);
      if (it != managers_.end())
        return *(it->second);
    }

    const std::unique_lock<std::shared_mutex> lock(mutex_);
    auto it = managers_.find(id);
    if (it == managers_.end())
    {
      // Clone before inserting, so a clone that throws leaves no entry behind for the lookup above to dereference
      std::unique_ptr<ManagerT> manager = prototype_->clone();
      it = managers_.emplace(id, std::move(manager)).first;
    }

    return *(it->second);
  }

  /** @brief The number of managers cloned so far: one per thread that has called get() */
  std::size_t size() const
  {
    const std::shared_lock<std::shared_mutex> lock(mutex_);
    return managers_.size();
  }

private:
  std::unique_ptr<const ManagerT> prototype_;
  mutable std::shared_mutex mutex_;
  mutable std::unordered_map<std::thread::id, std::unique_ptr<ManagerT>> managers_;
};

using PerThreadDiscreteContactManager = PerThreadContactManager<DiscreteContactManager>;
using PerThreadContinuousContactManager = PerThreadContactManager<ContinuousContactManager>;

}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_PER_THREAD_CONTACT_MANAGER_H
