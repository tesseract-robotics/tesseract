/**
 * @file object_cache.h
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date November 15, 2020
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
#ifndef TESSERACT_COMMON_OBJECT_CACHE_H
#define TESSERACT_COMMON_OBJECT_CACHE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <deque>
#include <memory>
#include <algorithm>
#include <mutex>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/sfinae_utils.h>

namespace tesseract_common
{
/** @brief Used to create a cache of objects
 *
 * CacheType needs the following methods
 * CacheType::Ptr clone() const;
 * int getRevision() const;
 * bool update(Const CacheType::ConstPtr&);  // optional
 * */
template <typename CacheType>
class CloneCache
{
public:
  CREATE_MEMBER_FUNC_SIGNATURE_CHECK(update, bool, const std::shared_ptr<const CacheType>&)
  CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(getRevision, int)
  CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(clone, std::shared_ptr<CacheType>)

  CloneCache(std::shared_ptr<CacheType> original, const long& cache_size = 5)
    : supports_update(has_member_func_signature_update<CacheType>::value)
    , original_(std::move(original))
    , cache_size_(static_cast<std::size_t>(cache_size))

  {
    // These methods are required
    static_assert(has_member_func_signature_getRevision<CacheType>::value,
                  "Class 'getRevision' function has incorrect signature");
    static_assert(has_member_func_signature_clone<CacheType>::value, "Class 'clone' function has incorrect signature");

    for (long i = 0; i < cache_size; i++)
      createClone();
  }

  const std::shared_ptr<CacheType>& operator->() { return original_; }

  /**
   * @brief Gets a clone of original_
   * @return A shared_ptr to a new clone of original_
   */
  std::shared_ptr<CacheType> clone()
  {
    if (!original_)
      return nullptr;

    if (cache_.empty())
    {
      std::shared_ptr<CacheType> cache = getClone();
      if (cache == nullptr)
        return nullptr;

      return cache;
    }

    // If the cache needs updating, then update it
    std::unique_lock<std::mutex> lock(cache_mutex_);
    if (cache_.back()->getRevision() != original_->getRevision())
    {
      // Update if possible
      std::shared_ptr<CacheType> t;
      if constexpr (has_member_func_signature_update<CacheType>::value)
      {
        cache_.back()->update(original_);
      }
      else
      {
        std::shared_ptr<CacheType> cache = getClone();
        if (cache == nullptr)
          return nullptr;

        cache_.back() = cache;
      }
      t = cache_.back();
      cache_.pop_back();
      return t;
    }

    std::shared_ptr<CacheType> t;
    t = cache_.back();
    cache_.pop_back();
    return t;
  }

  /**
   * @brief Set the cache size
   * @param size The size of the cache.
   */
  void setCacheSize(long size)
  {
    {
      std::unique_lock<std::mutex> lock(cache_mutex_);
      cache_size_ = static_cast<std::size_t>(size);
    }
    updateCache();
  }

  /**
   * @brief Get the set cache size
   * @return The set size of the cache.
   */
  long getCacheSize() const { return static_cast<long>(cache_size_); }

  /**
   * @brief Get the current size of the cache
   * @return The current size fo the cache
   */
  long getCurrentCacheSize() const
  {
    std::unique_lock<std::mutex> lock(cache_mutex_);
    return static_cast<long>(cache_.size());
  }

  /** @brief If original_ has changed it will update or rebuild the cache of objects */
  void updateCache()
  {
    std::unique_lock<std::mutex> lock(cache_mutex_);
    if (!original_)
      return;

    std::shared_ptr<CacheType> t;

    // Update all cached objects
    for (auto& cache : cache_)
    {
      if (cache->getRevision() != original_->getRevision())
      {
        // Update if possible
        if constexpr (has_member_func_signature_update<CacheType>::value)
        {
          cache->update(original_);
        }
        else
        {  // Update is not available to assign new clone
          cache = getClone();
        }
      }
    }

    // Prune nullptr
    cache_.erase(std::remove_if(cache_.begin(),
                                cache_.end(),
                                [](const std::shared_ptr<CacheType>& cache) { return (cache == nullptr); }),
                 cache_.end());

    while (cache_.size() < cache_size_)
    {
      CONSOLE_BRIDGE_logDebug("Adding clone to the cache. Current cache size: %i", cache_.size());
      std::shared_ptr<CacheType> clone = getClone();
      if (clone != nullptr)
        cache_.push_back(clone);
    }
  }

  const bool supports_update;

protected:
  void createClone()
  {
    if (original_ == nullptr)
      return;

    std::shared_ptr<CacheType> clone = getClone();
    if (clone == nullptr)
      return;

    // Lock cache
    std::unique_lock<std::mutex> lock(cache_mutex_);
    cache_.push_back(clone);
  }

  std::shared_ptr<CacheType> getClone() const
  {
    std::shared_ptr<CacheType> clone;
    try
    {
      clone = original_->clone();
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("Clone Cache failed to update cache with the following exception: %s", e.what());
      return nullptr;
    }
    return clone;
  }

  std::shared_ptr<CacheType> original_;

  /** @brief The assigned cache size */
  std::size_t cache_size_{ 5 };

  /** @brief A vector of cached objects */
  std::deque<std::shared_ptr<CacheType>> cache_;

  /** @brief The mutex used when reading and writing to cache_ */
  mutable std::mutex cache_mutex_;
};

}  // namespace tesseract_common
#endif
