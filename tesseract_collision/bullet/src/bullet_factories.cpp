/**
 * @file bullet_factories.h
 * @brief Factories for loading bullet contact managers as plugins
 *
 * @author Levi Armstrong
 * @date October 25, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_collision/bullet/bullet_factories.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>

namespace tesseract_collision::tesseract_collision_bullet
{
DiscreteContactManager::UPtr BulletDiscreteBVHManagerFactory::create(const std::string& name,
                                                                     const YAML::Node& /*config*/) const
{
  return std::make_unique<BulletDiscreteBVHManager>(name);
}

DiscreteContactManager::UPtr BulletDiscreteSimpleManagerFactory::create(const std::string& name,
                                                                        const YAML::Node& /*config*/) const
{
  return std::make_unique<BulletDiscreteSimpleManager>(name);
}

ContinuousContactManager::UPtr BulletCastBVHManagerFactory::create(const std::string& name,
                                                                   const YAML::Node& /*config*/) const
{
  return std::make_unique<BulletCastBVHManager>(name);
}

ContinuousContactManager::UPtr BulletCastSimpleManagerFactory::create(const std::string& name,
                                                                      const YAML::Node& /*config*/) const
{
  return std::make_unique<BulletCastSimpleManager>(name);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(BulletFactoriesAnchor)

}  // namespace tesseract_collision::tesseract_collision_bullet

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManagerFactory,
                                      BulletDiscreteBVHManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(
    tesseract_collision::tesseract_collision_bullet::BulletDiscreteSimpleManagerFactory,
    BulletDiscreteSimpleManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN(tesseract_collision::tesseract_collision_bullet::BulletCastBVHManagerFactory,
                                        BulletCastBVHManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN(tesseract_collision::tesseract_collision_bullet::BulletCastSimpleManagerFactory,
                                        BulletCastSimpleManagerFactory);
