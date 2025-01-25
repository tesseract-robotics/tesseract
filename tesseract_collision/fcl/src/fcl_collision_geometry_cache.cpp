/**
 * @file fcl_collision_geometry_cache.cpp
 * @brief This is a static cache mapping tesseract geometry shared pointers to fcl collision geometry to avoid
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

#include <tesseract_collision/fcl/fcl_collision_geometry_cache.h>
#include <tesseract_geometry/geometry.h>
#include <mutex>

namespace tesseract_collision::tesseract_collision_fcl
{
// Static member definitions
std::map<std::shared_ptr<const tesseract_geometry::Geometry>, std::shared_ptr<fcl::CollisionGeometryd>>
    FCLCollisionGeometryCache::cache_;
std::shared_mutex FCLCollisionGeometryCache::mutex_;

void FCLCollisionGeometryCache::insert(const std::shared_ptr<const tesseract_geometry::Geometry>& key,
                                       const std::shared_ptr<fcl::CollisionGeometryd>& value)
{
  std::unique_lock lock(mutex_);
  cache_[key] = value;
}

std::shared_ptr<fcl::CollisionGeometryd>
FCLCollisionGeometryCache::get(const std::shared_ptr<const tesseract_geometry::Geometry>& key)
{
  std::shared_lock lock(mutex_);
  auto it = cache_.find(key);
  return ((it != cache_.end()) ? it->second : nullptr);
}

}  // namespace tesseract_collision::tesseract_collision_fcl
