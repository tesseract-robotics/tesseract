/**
 * @file process_environment_cache.h
 * @brief A process environment cache
 *
 * @author Levi Armstrong
 * @date December 3, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_ENVIRONMENT_CACHE_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_ENVIRONMENT_CACHE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <thread>
#include <mutex>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>

namespace tesseract_planning
{
class EnvironmentCache
{
public:
  using Ptr = std::shared_ptr<EnvironmentCache>;
  using ConstPtr = std::shared_ptr<const EnvironmentCache>;

  EnvironmentCache() = default;
  virtual ~EnvironmentCache() = default;
  EnvironmentCache(const EnvironmentCache&) = delete;
  EnvironmentCache& operator=(const EnvironmentCache&) = delete;
  EnvironmentCache(EnvironmentCache&&) = delete;
  EnvironmentCache& operator=(EnvironmentCache&&) = delete;

  /**
   * @brief Set the cache size used to hold tesseract objects for motion planning
   * @param size The size of the cache.
   */
  virtual void setCacheSize(long size) = 0;

  /**
   * @brief Get the cache size used to hold tesseract objects for motion planning
   * @return The size of the cache.
   */
  virtual long getCacheSize() const = 0;

  /** @brief If the environment has changed it will rebuild the cache of tesseract objects */
  virtual void refreshCache() = 0;

  /**
   * @brief This will pop an Environment object from the queue
   * @details This will first call refreshCache to ensure it has an updated tesseract then proceed
   */
  virtual tesseract_environment::Environment::Ptr getCachedEnvironment() = 0;
};

class ProcessEnvironmentCache : public EnvironmentCache
{
public:
  using Ptr = std::shared_ptr<ProcessEnvironmentCache>;
  using ConstPtr = std::shared_ptr<const ProcessEnvironmentCache>;

  ProcessEnvironmentCache(tesseract_environment::Environment::ConstPtr env, std::size_t cache_size = 5);

  /**
   * @brief Set the cache size used to hold tesseract objects for motion planning
   * @param size The size of the cache.
   */
  void setCacheSize(long size) override;

  /**
   * @brief Get the cache size used to hold tesseract objects for motion planning
   * @return The size of the cache.
   */
  long getCacheSize() const override;

  /** @brief If the environment has changed it will rebuild the cache of tesseract objects */
  void refreshCache() override;

  /**
   * @brief This will pop an Environment object from the queue
   * @details This will first call refreshCache to ensure it has an updated tesseract then proceed
   */
  tesseract_environment::Environment::Ptr getCachedEnvironment() override;

protected:
  /** @brief The tesseract_object used to create the cache */
  tesseract_environment::Environment::ConstPtr env_;

  /** @brief The environment revision number at the time the cache was populated */
  int cache_env_revision_{ 0 };

  /** @brief The assigned cache size */
  std::size_t cache_size_{ 5 };

  /** @brief A vector of cached Tesseact objects */
  std::deque<tesseract_environment::Environment::Ptr> cache_;

  /** @brief The mutex used when reading and writing to cache_ */
  mutable std::shared_mutex cache_mutex_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_ENVIRONMENT_CACHE_H
