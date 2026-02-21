/**
 * @file profile_plugin_factory_unit.cpp
 * @brief Tesseract profile plugin factory test
 *
 * @author Levi Armstrong
 * @date Feb 4, 2025
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/profile.h>
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>
#include <tesseract_common/utils.h>
#include "test_profile.h"

using namespace tesseract::common;

TEST(TesseractCommonProfileFactoryUnit, PluginFactoryAPIUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;

  ProfilePluginFactory factory;
  EXPECT_TRUE(factory.getSearchPaths().empty());
  EXPECT_EQ(factory.getSearchPaths().size(), 0);
  EXPECT_TRUE(factory.getSearchLibraries().empty());
  EXPECT_EQ(factory.getSearchLibraries().size(), 0);
  EXPECT_EQ(factory.getPlugins().size(), 0);
  EXPECT_FALSE(factory.hasPlugins());
  EXPECT_TRUE(factory.getFactoryData() == nullptr);
  EXPECT_TRUE(factory.getProfileDictionary().getAllProfileEntries().empty());

  factory.addSearchPath("/usr/local/lib");
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_EQ(factory.getSearchLibraries().size(), 0);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_EQ(factory.getSearchLibraries().size(), 1);

  factory.clearSearchPaths();
  EXPECT_EQ(factory.getSearchPaths().size(), 0);
  EXPECT_EQ(factory.getSearchLibraries().size(), 1);

  factory.addSearchPath("/usr/local/lib");
  factory.clearSearchLibraries();
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_EQ(factory.getSearchLibraries().size(), 0);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_EQ(factory.getSearchLibraries().size(), 1);

  auto factory_data = std::make_shared<ProfileFactoryData>();
  factory.setFactoryData(factory_data);
  EXPECT_EQ(factory_data, factory.getFactoryData());

  {
    std::map<std::string, tesseract::common::PluginInfoMap> map = factory.getPlugins();
    const std::string ns{ "TesseractCollisionCost" };
    const std::string name{ "freespace" };
    const std::string default_name{ "default" };
    const std::string class_name{ "TestProfileFactory" };
    EXPECT_TRUE(map.find(ns) == map.end());

    tesseract::common::PluginInfo pi;
    pi.class_name = class_name;
    factory.addPlugin(ns, name, pi);
    EXPECT_EQ(factory.getPlugins().size(), 1);
    EXPECT_TRUE(factory.hasPlugins());

    map = factory.getPlugins();
    EXPECT_TRUE(map.find(ns) != map.end());
    EXPECT_TRUE(map.at(ns).find(name) != map.at(ns).end());
    EXPECT_EQ(map.find(ns)->second.size(), 1);

    tesseract::common::PluginInfo pi2;
    pi2.class_name = class_name;
    factory.addPlugin(ns, default_name, pi2);
    EXPECT_EQ(factory.getPlugins().size(), 1);

    map = factory.getPlugins();
    EXPECT_TRUE(map.find(ns) != map.end());
    EXPECT_TRUE(map.at(ns).find(default_name) != map.at(ns).end());
    EXPECT_EQ(map.find(ns)->second.size(), 2);

    EXPECT_ANY_THROW(factory.removePlugin("ns_does_not_exist", default_name));
    EXPECT_ANY_THROW(factory.removePlugin(ns, "profile_does_not_exist"));  // NOLINT
    EXPECT_TRUE(factory.create("ns_does_not_exist", default_name) == nullptr);
    EXPECT_TRUE(factory.create(ns, "profile_does_not_exist") == nullptr);

    factory.removePlugin(ns, default_name);

    map = factory.getPlugins();
    EXPECT_TRUE(map.find(ns) != map.end());
    EXPECT_EQ(factory.getPlugins().size(), 1);
    EXPECT_EQ(map.find(ns)->second.size(), 1);

    factory.removePlugin(ns, name);
    map = factory.getPlugins();
    EXPECT_TRUE(map.find(ns) == map.end());
    EXPECT_EQ(factory.getPlugins().size(), 0);
  }
}

TEST(TesseractCommonProfileFactoryUnit, LoadProfilePluginInfoUnit)  // NOLINT
{
  const std::string lib_dir = std::string(TEST_PLUGIN_DIR);
  const std::filesystem::path config_filepath = std::filesystem::path(tesseract::common::getTempPath()) / "profile_"
                                                                                                          "plugins_"
                                                                                                          "export.yaml";
  tesseract::common::GeneralResourceLocator locator;

  std::string yaml_string = R"(
profile_plugins:
  search_paths:
    - )" + lib_dir + R"(
  search_libraries:
    - tesseract_common_test_plugins
  profiles:
    TrajOptCollisionCost:
      DiscreteCollisionProfile:
        class: TestProfileFactory
        config:
          enabled: true
      DiscreteCollisionProfile2:
        class: TestProfileFactory
        config:
          enabled: false
    TrajOptCollisionConstraint:
      DiscreteCollisionProfile:
        class: TestProfileFactory
        config:
          enabled: false
      DiscreteCollisionProfile2:
        class: TestProfileFactory
        config:
          enabled: true)";

  {  // Load via yaml string
    ProfilePluginFactory factory(yaml_string, locator);
    EXPECT_FALSE(factory.getProfileDictionary().getAllProfileEntries().empty());
    auto map = factory.getPlugins();
    EXPECT_EQ(map.size(), 2);
    EXPECT_TRUE(map.find("TrajOptCollisionCost") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_EQ(map.find("TrajOptCollisionCost")->second.size(), 2);

    EXPECT_TRUE(map.find("TrajOptCollisionConstraint") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_EQ(map.find("TrajOptCollisionConstraint")->second.size(), 2);

    std::shared_ptr<Profile> plugin = factory.create("TrajOptCollisionCost", "DiscreteCollisionProfile");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_TRUE(std::dynamic_pointer_cast<TestProfile>(plugin)->enabled);

    plugin = factory.create("TrajOptCollisionCost", "DiscreteCollisionProfile2");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_FALSE(std::dynamic_pointer_cast<TestProfile>(plugin)->enabled);

    plugin = factory.create("TrajOptCollisionConstraint", "DiscreteCollisionProfile");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_FALSE(std::dynamic_pointer_cast<TestProfile>(plugin)->enabled);

    plugin = factory.create("TrajOptCollisionConstraint", "DiscreteCollisionProfile2");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_TRUE(std::dynamic_pointer_cast<TestProfile>(plugin)->enabled);

    factory.saveConfig(config_filepath);
  }

  {  // Load via YAML::Node
    ProfilePluginFactory factory(config_filepath, locator);
    auto map = factory.getPlugins();
    EXPECT_EQ(map.size(), 2);
    EXPECT_TRUE(map.find("TrajOptCollisionCost") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_EQ(map.find("TrajOptCollisionCost")->second.size(), 2);

    EXPECT_TRUE(map.find("TrajOptCollisionConstraint") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_EQ(map.find("TrajOptCollisionConstraint")->second.size(), 2);
  }

  {  // Load via YAML::Node
    YAML::Node config = tesseract::common::loadYamlString(yaml_string, locator);
    ProfilePluginFactory factory(config, locator);
    auto map = factory.getPlugins();
    EXPECT_EQ(map.size(), 2);
    EXPECT_TRUE(map.find("TrajOptCollisionCost") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_EQ(map.find("TrajOptCollisionCost")->second.size(), 2);

    EXPECT_TRUE(map.find("TrajOptCollisionConstraint") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_EQ(map.find("TrajOptCollisionConstraint")->second.size(), 2);
  }

  {  // Load via YAML::Node
    YAML::Node config_yaml = tesseract::common::loadYamlString(yaml_string, locator);
    auto config =
        config_yaml[tesseract::common::ProfilesPluginInfo::CONFIG_KEY].as<tesseract::common::ProfilesPluginInfo>();
    ProfilePluginFactory factory(config);
    auto map = factory.getPlugins();
    EXPECT_EQ(map.size(), 2);
    EXPECT_TRUE(map.find("TrajOptCollisionCost") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_TRUE(map.at("TrajOptCollisionCost").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionCost").end());
    EXPECT_EQ(map.find("TrajOptCollisionCost")->second.size(), 2);

    EXPECT_TRUE(map.find("TrajOptCollisionConstraint") != map.end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_TRUE(map.at("TrajOptCollisionConstraint").find("DiscreteCollisionProfile2") !=
                map.at("TrajOptCollisionConstraint").end());
    EXPECT_EQ(map.find("TrajOptCollisionConstraint")->second.size(), 2);
  }

  {  // missing entry
    YAML::Node config = tesseract::common::loadYamlString(yaml_string, locator);
    auto plugin_yaml = config["profile_plugins"]["profiles"]["TrajOptCollisionCost"];
    plugin_yaml.remove("DiscreteCollisionProfile2");

    ProfilePluginFactory factory(config, locator);
    auto plugin = factory.create("TrajOptCollisionCost", "DiscreteCollisionProfile2");
    EXPECT_TRUE(plugin == nullptr);
  }
  {  // missing class
    YAML::Node config = tesseract::common::loadYamlString(yaml_string, locator);
    auto plugin_yaml = config["profile_plugins"]["profiles"]["TrajOptCollisionCost"]["DiscreteCollisionProfile2"];
    plugin_yaml.remove("class");

    EXPECT_ANY_THROW(ProfilePluginFactory factory(config, locator));  // NOLINT
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
