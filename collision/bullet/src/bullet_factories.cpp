/**
 * @file bullet_factories.h
 * @brief Factories for loading bullet contact managers as plugins
 *
 * @author Levi Armstrong
 * @date October 25, 2021
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_factories.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/tesseract_collision_configuration.h>

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

namespace YAML
{
template <>
struct convert<tesseract::collision::TesseractCollisionConfigurationInfo>
{
  static bool decode(const Node& node, tesseract::collision::TesseractCollisionConfigurationInfo& rhs)
  {
    if (node.IsNull())
    {
      rhs = tesseract::collision::TesseractCollisionConfigurationInfo();
      return true;
    }

    bool share_pool_allocators{ false };
    if (YAML::Node n = node["share_pool_allocators"])
      share_pool_allocators = n.as<bool>();

    rhs = tesseract::collision::TesseractCollisionConfigurationInfo(false, share_pool_allocators);

    if (YAML::Node n = node["max_persistent_manifold_pool_size"])
      rhs.m_defaultMaxPersistentManifoldPoolSize = n.as<int>();

    if (YAML::Node n = node["max_collision_algorithm_pool_size"])
      rhs.m_defaultMaxCollisionAlgorithmPoolSize = n.as<int>();

    if (YAML::Node n = node["max_custom_collision_algorithm_element_size"])
      rhs.m_customCollisionAlgorithmMaxElementSize = n.as<int>();

    if (YAML::Node n = node["use_epa_penetration_algorithm"])
      rhs.m_useEpaPenetrationAlgorithm = static_cast<int>(n.as<bool>());

    rhs.createPoolAllocators();
    return true;
  }

  static tesseract::common::PropertyTree schema()
  {
    // clang-format off
    return tesseract::common::PropertyTreeBuilder()
        .attribute(tesseract::common::property_attribute::TYPE, tesseract::common::property_type::CONTAINER)
        .boolean("share_pool_allocators").done()
        .integer("max_persistent_manifold_pool_size").minimum(0).maximum(4096).done()
        .integer("max_collision_algorithm_pool_size").minimum(0).maximum(4096).done()
        .integer("max_custom_collision_algorithm_element_size").minimum(0).maximum(4096).done()
        .boolean("use_epa_penetration_algorithm").done()
        .build();
    // clang-format on
  }
};
}  // namespace YAML

namespace tesseract::collision
{
tesseract::common::PropertyTree BulletDiscreteBVHManagerFactory::schema() const
{
  return YAML::convert<TesseractCollisionConfigurationInfo>::schema();
}

tesseract::common::PropertyTree BulletDiscreteSimpleManagerFactory::schema() const
{
  return YAML::convert<TesseractCollisionConfigurationInfo>::schema();
}

tesseract::common::PropertyTree BulletCastBVHManagerFactory::schema() const
{
  return YAML::convert<TesseractCollisionConfigurationInfo>::schema();
}

tesseract::common::PropertyTree BulletCastSimpleManagerFactory::schema() const
{
  return YAML::convert<TesseractCollisionConfigurationInfo>::schema();
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
BulletDiscreteBVHManagerFactory::create(const std::string& name, const YAML::Node& config) const
{
  return std::make_unique<BulletDiscreteBVHManager>(name, config.as<TesseractCollisionConfigurationInfo>());
}

std::unique_ptr<DiscreteContactManager> BulletDiscreteSimpleManagerFactory::create(const std::string& name,
                                                                                   const YAML::Node& config) const
{
  return std::make_unique<BulletDiscreteSimpleManager>(name, config.as<TesseractCollisionConfigurationInfo>());
}

std::unique_ptr<ContinuousContactManager> BulletCastBVHManagerFactory::create(const std::string& name,
                                                                              const YAML::Node& config) const
{
  return std::make_unique<BulletCastBVHManager>(name, config.as<TesseractCollisionConfigurationInfo>());
}

std::unique_ptr<ContinuousContactManager> BulletCastSimpleManagerFactory::create(const std::string& name,
                                                                                 const YAML::Node& config) const
{
  return std::make_unique<BulletCastSimpleManager>(name, config.as<TesseractCollisionConfigurationInfo>());
}

PLUGIN_ANCHOR_IMPL(BulletFactoriesAnchor)

}  // namespace tesseract::collision

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(tesseract::collision::BulletDiscreteBVHManagerFactory,
                                      BulletDiscreteBVHManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(tesseract::collision::BulletDiscreteSimpleManagerFactory,
                                      BulletDiscreteSimpleManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN(tesseract::collision::BulletCastBVHManagerFactory, BulletCastBVHManagerFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_CONTINUOUS_MANAGER_PLUGIN(tesseract::collision::BulletCastSimpleManagerFactory,
                                        BulletCastSimpleManagerFactory);

TESSERACT_SCHEMA_REGISTER(BulletDiscreteBVHManagerFactory,
                          YAML::convert<tesseract::collision::TesseractCollisionConfigurationInfo>::schema);
TESSERACT_SCHEMA_REGISTER(BulletDiscreteSimpleManagerFactory,
                          YAML::convert<tesseract::collision::TesseractCollisionConfigurationInfo>::schema);
TESSERACT_SCHEMA_REGISTER(BulletCastBVHManagerFactory,
                          YAML::convert<tesseract::collision::TesseractCollisionConfigurationInfo>::schema);
TESSERACT_SCHEMA_REGISTER(BulletCastSimpleManagerFactory,
                          YAML::convert<tesseract::collision::TesseractCollisionConfigurationInfo>::schema);

TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::collision::DiscreteContactManagerFactory,
                                       BulletDiscreteBVHManagerFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::collision::DiscreteContactManagerFactory,
                                       BulletDiscreteSimpleManagerFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::collision::ContinuousContactManagerFactory,
                                       BulletCastBVHManagerFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::collision::ContinuousContactManagerFactory,
                                       BulletCastSimpleManagerFactory);