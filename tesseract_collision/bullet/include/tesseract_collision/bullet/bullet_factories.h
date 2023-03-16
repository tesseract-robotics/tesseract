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
#ifndef TESSERACT_COLLISION_BULLET_BULLET_FACTORIES_H
#define TESSERACT_COLLISION_BULLET_BULLET_FACTORIES_H

#include <tesseract_collision/core/contact_managers_plugin_factory.h>

namespace tesseract_collision::tesseract_collision_bullet
{
/**
 * @brief The yaml config for each of the factories below is the same.
 * @details
 * The config and its parameters shown below are optional
 * The values shown below are the default that will be used.
 *
 * The current defaults will result in 7MB being allocated for every contact manager created.
 * If share_pool_allocators is set to true then this 7MB is shared between it and any clones created.
 *
 * Example Yaml Config:
 *
 *    plugins:
 *      BulletDiscreteBVHManager:
 *        class: BulletDiscreteBVHManagerFactory
 *        config:
 *          share_pool_allocators: false
 *          max_persistent_manifold_pool_size: 4096
 *          max_collision_algorithm_pool_size: 4096
 */
class BulletDiscreteBVHManagerFactory : public DiscreteContactManagerFactory
{
public:
  DiscreteContactManager::UPtr create(const std::string& name, const YAML::Node& config) const override final;
};

class BulletDiscreteSimpleManagerFactory : public DiscreteContactManagerFactory
{
public:
  DiscreteContactManager::UPtr create(const std::string& name, const YAML::Node& config) const override final;
};

class BulletCastBVHManagerFactory : public ContinuousContactManagerFactory
{
public:
  ContinuousContactManager::UPtr create(const std::string& name, const YAML::Node& config) const override final;
};

class BulletCastSimpleManagerFactory : public ContinuousContactManagerFactory
{
public:
  ContinuousContactManager::UPtr create(const std::string& name, const YAML::Node& config) const override final;
};

TESSERACT_PLUGIN_ANCHOR_DECL(BulletFactoriesAnchor)

}  // namespace tesseract_collision::tesseract_collision_bullet
#endif  // TESSERACT_COLLISION_BULLET_BULLET_FACTORIES_H
