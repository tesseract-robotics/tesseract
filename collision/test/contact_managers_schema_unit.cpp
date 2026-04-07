/**
 * @file contact_managers_schema_unit.cpp
 * @brief Tests for ContactManagersPluginInfo schema validation with derived type support
 *
 * @author Levi Armstrong
 * @date April 4, 2026
 *
 * @copyright Copyright (c) 2026, Levi Armstrong
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
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/yaml_extensions.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/collision/bullet/bullet_factories.h>
#include <tesseract/collision/fcl/fcl_factories.h>
#include <boost_plugin_loader/utils.h>

using namespace tesseract::common;

// Force-link the factory libraries so their TESSERACT_SCHEMA_REGISTER macros run
static const auto& bullet_anchor = tesseract::collision::BulletFactoriesAnchor();
static const auto& fcl_anchor = tesseract::collision::FCLFactoriesAnchor();

TEST(ContactManagersSchemaUnit, ValidFullConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    search_paths:
      - /usr/local/lib
    search_libraries:
      - tesseract_collision_bullet_factories
    discrete_plugins:
      default: BulletDiscreteBVHManager
      plugins:
        BulletDiscreteBVHManager:
          class: BulletDiscreteBVHManagerFactory
          config:
            share_pool_allocators: false
        BulletDiscreteSimpleManager:
          class: BulletDiscreteSimpleManagerFactory
        FCLDiscreteBVHManager:
          class: FCLDiscreteBVHManagerFactory
    continuous_plugins:
      default: BulletCastBVHManager
      plugins:
        BulletCastBVHManager:
          class: BulletCastBVHManagerFactory
        BulletCastSimpleManager:
          class: BulletCastSimpleManagerFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

TEST(ContactManagersSchemaUnit, ValidBulletWithConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: BulletDiscreteBVHManager
      plugins:
        BulletDiscreteBVHManager:
          class: BulletDiscreteBVHManagerFactory
          config:
            share_pool_allocators: true
            max_persistent_manifold_pool_size: 1024
            max_collision_algorithm_pool_size: 1024
            use_epa_penetration_algorithm: false
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

TEST(ContactManagersSchemaUnit, ValidFCLNoConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: FCLDiscreteBVHManager
      plugins:
        FCLDiscreteBVHManager:
          class: FCLDiscreteBVHManagerFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

TEST(ContactManagersSchemaUnit, ValidEmptyConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config;  // null/empty
  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

TEST(ContactManagersSchemaUnit, InvalidDerivedType)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // Use a class name that is NOT registered as a derived type
  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: FakeManager
      plugins:
        FakeManager:
          class: NonExistentFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_FALSE(errors.empty());

  // Should mention that the type doesn't derive from the base
  bool found_derive_error = false;
  for (const auto& e : errors)
  {
    if (e.find("does not derive from") != std::string::npos || e.find("no schema registry") != std::string::npos)
    {
      found_derive_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_derive_error) << "Expected a derived-type or schema-not-found error";
}

TEST(ContactManagersSchemaUnit, InvalidDiscretePluginInContinuous)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // BulletDiscreteBVHManagerFactory is registered under DiscreteContactManagerFactory,
  // NOT ContinuousContactManagerFactory — should fail validation
  YAML::Node config = YAML::Load(R"(
    continuous_plugins:
      default: WrongManager
      plugins:
        WrongManager:
          class: BulletDiscreteBVHManagerFactory
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_FALSE(errors.empty());

  bool found_derive_error = false;
  for (const auto& e : errors)
  {
    if (e.find("does not derive from") != std::string::npos)
    {
      found_derive_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_derive_error) << "Expected a derived-type mismatch error";
}

TEST(ContactManagersSchemaUnit, InvalidMissingClassField)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // Plugin entry without the required "class" field
  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: BadPlugin
      plugins:
        BadPlugin:
          config:
            share_pool_allocators: true
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_FALSE(errors.empty());

  bool found_class_error = false;
  for (const auto& e : errors)
  {
    if (e.find("class") != std::string::npos)
    {
      found_class_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_class_error) << "Expected an error about missing 'class' field";
}

TEST(ContactManagersSchemaUnit, InvalidBulletConfigValue)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // The bullet config expects integers for pool sizes, not strings
  YAML::Node config = YAML::Load(R"(
    discrete_plugins:
      default: BulletDiscreteBVHManager
      plugins:
        BulletDiscreteBVHManager:
          class: BulletDiscreteBVHManagerFactory
          config:
            max_persistent_manifold_pool_size: not_a_number
  )");

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_FALSE(errors.empty());
}

TEST(ContactManagersSchemaUnit, ValidMinimalDiscretePluginConfig)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config;
  YAML::Node discrete_plugins(YAML::NodeType::Map);
  discrete_plugins["default"] = "BulletDiscreteSimpleManager";

  YAML::Node plugins(YAML::NodeType::Map);
  YAML::Node plugin_info(YAML::NodeType::Map);
  plugin_info["class"] = "BulletDiscreteSimpleManagerFactory";
  plugins["BulletDiscreteSimpleManager"] = plugin_info;

  discrete_plugins["plugins"] = plugins;
  config["discrete_plugins"] = discrete_plugins;

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

TEST(ContactManagersSchemaUnit, InvalidPluginsSectionWrongType)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  YAML::Node config;
  YAML::Node discrete_plugins(YAML::NodeType::Map);
  discrete_plugins["default"] = "BulletDiscreteSimpleManager";
  discrete_plugins["plugins"] = "not_a_map";
  config["discrete_plugins"] = discrete_plugins;

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_FALSE(errors.empty());

  bool found_plugins_error = false;
  for (const auto& e : errors)
  {
    if (e.find("plugins") != std::string::npos)
    {
      found_plugins_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_plugins_error) << "Expected an error about 'plugins' having the wrong type";
}

TEST(ContactManagersSchemaUnit, ValidFromYamlFile)  // NOLINT
{
  auto schema = YAML::convert<ContactManagersPluginInfo>::schema();

  // Use the existing test YAML file
  std::filesystem::path file_path(__FILE__);
  std::filesystem::path config_path = file_path.parent_path() / "contact_manager_plugins.yaml";

  YAML::Node full_config = YAML::LoadFile(config_path.string());
  YAML::Node config = full_config["contact_manager_plugins"];

  schema.mergeConfig(config);
  auto errors = schema.validate();
  EXPECT_TRUE(errors.empty()) << [&] {
    std::string msg;
    for (const auto& e : errors)
      msg += e + "\n";
    return msg;
  }();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
