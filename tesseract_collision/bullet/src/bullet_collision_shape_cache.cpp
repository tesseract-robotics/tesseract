/**
 * @file bullet_collision_shape_cache.cpp
 * @brief This is a static cache mapping tesseract geometry shared pointers to bullet collision shapes to avoid
 * recreating the same collision object
 *
 * @author Levi Armstrong
 * @date January 25, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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

#include <tesseract_collision/bullet/bullet_collision_shape_cache.h>
#include <tesseract_geometry/geometry.h>
#include <mutex>

namespace tesseract_collision::tesseract_collision_bullet
{
// Static member definitions
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::map<boost::uuids::uuid, std::weak_ptr<BulletCollisionShape>> BulletCollisionShapeCache::cache_;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::mutex BulletCollisionShapeCache::mutex_;

BulletCollisionShape::BulletCollisionShape(std::shared_ptr<btCollisionShape> top_level_)
  : top_level(std::move(top_level_))
{
}

void BulletCollisionShapeCache::insert(const std::shared_ptr<const tesseract_geometry::Geometry>& key,
                                       const std::shared_ptr<BulletCollisionShape>& value)
{
  assert(!key->getUUID().is_nil());
  std::scoped_lock lock(mutex_);
  cache_[key->getUUID()] = value;
}

std::shared_ptr<BulletCollisionShape>
BulletCollisionShapeCache::get(const std::shared_ptr<const tesseract_geometry::Geometry>& key)
{
  assert(!key->getUUID().is_nil());
  std::scoped_lock lock(mutex_);
  auto it = cache_.find(key->getUUID());
  if (it != cache_.end())
  {
    std::shared_ptr<BulletCollisionShape> collision_shape = it->second.lock();
    if (collision_shape != nullptr)
      return collision_shape;

    cache_.erase(key->getUUID());
  }
  return nullptr;
}

void BulletCollisionShapeCache::prune()
{
  std::scoped_lock lock(mutex_);
  for (auto it = cache_.begin(); it != cache_.end();)
  {
    if (it->second.expired())
      it = cache_.erase(it);
    else
      ++it;
  }
}

}  // namespace tesseract_collision::tesseract_collision_bullet
