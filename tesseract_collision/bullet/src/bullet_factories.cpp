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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/bullet_factories.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/tesseract_collision_configuration.h>

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

namespace tesseract_collision::tesseract_collision_bullet
{
TesseractCollisionConfigurationInfo getConfigInfo(const YAML::Node& config)
{
  if (config.IsNull())
    return {};

  bool share_pool_allocators{ false };
  if (YAML::Node n = config["share_pool_allocators"])
    share_pool_allocators = n.as<bool>();

  TesseractCollisionConfigurationInfo config_info(false, share_pool_allocators);

  if (YAML::Node n = config["max_persistent_manifold_pool_size"])
    config_info.m_defaultMaxPersistentManifoldPoolSize = n.as<int>();

  if (YAML::Node n = config["max_collision_algorithm_pool_size"])
    config_info.m_defaultMaxCollisionAlgorithmPoolSize = n.as<int>();

  config_info.createPoolAllocators();
  return config_info;
}

std::unique_ptr<tesseract_collision::DiscreteContactManager>
BulletDiscreteBVHManagerFactory::create(const std::string& name, const YAML::Node& config) const
{
  return std::make_unique<BulletDiscreteBVHManager>(name, getConfigInfo(config));
}

std::unique_ptr<DiscreteContactManager> BulletDiscreteSimpleManagerFactory::create(const std::string& name,
                                                                                   const YAML::Node& config) const
{
  return std::make_unique<BulletDiscreteSimpleManager>(name, getConfigInfo(config));
}

std::unique_ptr<ContinuousContactManager> BulletCastBVHManagerFactory::create(const std::string& name,
                                                                              const YAML::Node& config) const
{
  return std::make_unique<BulletCastBVHManager>(name, getConfigInfo(config));
}

std::unique_ptr<ContinuousContactManager> BulletCastSimpleManagerFactory::create(const std::string& name,
                                                                                 const YAML::Node& config) const
{
  return std::make_unique<BulletCastSimpleManager>(name, getConfigInfo(config));
}

PLUGIN_ANCHOR_IMPL(BulletFactoriesAnchor)

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
