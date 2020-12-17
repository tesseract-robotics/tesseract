/**
 * @file process_environment_cache.cpp
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

#include <tesseract_process_managers/core/process_environment_cache.h>

namespace tesseract_planning
{
ProcessEnvironmentCache::ProcessEnvironmentCache(tesseract_environment::Environment::ConstPtr env,
                                                 std::size_t cache_size)
  : env_(std::move(env)), cache_size_(cache_size)
{
}

void ProcessEnvironmentCache::setCacheSize(long size)
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  cache_size_ = static_cast<std::size_t>(size);
}

long ProcessEnvironmentCache::getCacheSize() const { return static_cast<long>(cache_size_); }

void ProcessEnvironmentCache::refreshCache()
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::Ptr env;

  int rev = env_->getRevision();
  if (rev != cache_env_revision_ || cache_.empty())
  {
    env = env_->clone();
    cache_env_revision_ = rev;
  }

  if (env != nullptr)
  {
    cache_.clear();
    for (std::size_t i = 0; i < cache_size_; ++i)
      cache_.push_back(env->clone());
  }
  else if (cache_.size() <= 2)
  {
    for (std::size_t i = (cache_.size() - 1); i < cache_size_; ++i)
      cache_.push_back(cache_.front()->clone());
  }
}

tesseract_environment::Environment::Ptr ProcessEnvironmentCache::getCachedEnvironment()
{
  // This is to make sure the cached items are updated if needed
  refreshCache();

  tesseract_environment::EnvState current_state;
  current_state = *(env_->getCurrentState());

  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::Ptr t = cache_.back();

  // Update to the current joint values
  t->setState(current_state.joints);

  cache_.pop_back();

  return t;
}
}  // namespace tesseract_planning
