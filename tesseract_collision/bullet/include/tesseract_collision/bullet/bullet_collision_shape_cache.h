/**
 * @file bullet_collision_shape_cache.h
 * @brief This is a static cache mapping tesseract geometry shared pointers to bullet collision shapes to avoid
 * recreating the same collision object
 *
 * @author Levi Armstrong
 * @date January 25, 2025
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
#ifndef TESSERACT_COLLISION_BULLET_COLLISION_SHAPE_CACHE_H
#define TESSERACT_COLLISION_BULLET_COLLISION_SHAPE_CACHE_H

#include <map>
#include <memory>
#include <vector>
#include <mutex>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/fwd.h>

namespace tesseract_collision::tesseract_collision_bullet
{
struct BulletCollisionShape
{
  BulletCollisionShape() = default;
  BulletCollisionShape(std::shared_ptr<btCollisionShape> top_level_);

  std::shared_ptr<btCollisionShape> top_level;
  std::vector<std::shared_ptr<btCollisionShape>> children;
};

class BulletCollisionShapeCache
{
public:
  /**
   * @brief Insert a new entry into the cache
   * @param key The cache key
   * @param value The value to store
   */
  static void insert(const std::shared_ptr<const tesseract_geometry::Geometry>& key,
                     const std::shared_ptr<BulletCollisionShape>& value);

  /**
   * @brief Retrieve the cache entry by key
   * @param key The cache key
   * @return If key exists the entry is returned, otherwise a nullptr is returned
   */
  static std::shared_ptr<BulletCollisionShape> get(const std::shared_ptr<const tesseract_geometry::Geometry>& key);

  /** @brief Remove any entries which are no longer valid */
  static void prune();

private:
  /** @brief The static cache */
  static std::map<boost::uuids::uuid, std::weak_ptr<BulletCollisionShape>> cache_;  // NOLINT
  /** @brief The shared mutex for thread safety */
  static std::mutex mutex_;  // NOLINT
};
}  // namespace tesseract_collision::tesseract_collision_bullet

#endif  // TESSERACT_COLLISION_BULLET_COLLISION_SHAPE_CACHE_H
