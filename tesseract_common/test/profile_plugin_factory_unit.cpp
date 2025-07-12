/**
 * @file profile_plugin_factory_unit.cpp
 * @brief Tesseract profile plugin factory test
 *
 * @author Levi Armstrong
 * @date Feb 4, 2025
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/utils.h>

using namespace tesseract_common;

TEST(TesseractCommonProfileFactoryUnit, PluginFactoryAPIUnit)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;

  ProfilePluginFactory factory;
  EXPECT_FALSE(factory.getSearchPaths().empty());
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_FALSE(factory.getSearchLibraries().empty());
  EXPECT_EQ(factory.getSearchLibraries().size(), 4);
  EXPECT_EQ(factory.getPlugins().size(), 0);
  EXPECT_TRUE(factory.getProfileDictionary().getAllProfileEntries().empty());

  factory.addSearchPath("/usr/local/lib");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 4);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 5);

  {
    std::map<std::string, tesseract_common::PluginInfoMap> map = factory.getPlugins();
    const std::string ns{ "TesseractCollisionCost" };
    const std::string name{ "freespace" };
    const std::string default_name{ "default" };
    const std::string class_name{ "DiscreteCollisionProfileFactory" };
    EXPECT_TRUE(map.find(ns) == map.end());

    tesseract_common::PluginInfo pi;
    pi.class_name = class_name;
    factory.addPlugin(ns, name, pi);
    EXPECT_EQ(factory.getPlugins().size(), 1);

    map = factory.getPlugins();
    EXPECT_TRUE(map.find(ns) != map.end());
    EXPECT_TRUE(map.at(ns).find(name) != map.at(ns).end());
    EXPECT_EQ(map.find(ns)->second.size(), 1);

    tesseract_common::PluginInfo pi2;
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
  }
}

TEST(TesseractCommonProfileFactoryUnit, LoadProfilePluginInfoUnit)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;

  std::string yaml_string = R"(profile_plugins:
                                 search_paths:
                                   - /usr/local/lib
                                 search_libraries:
                                   - profile_factories
                                 profiles:
                                   TrajOptCollisionCost:
                                     DiscreteCollisionProfile:
                                       class: DiscreteCollisionProfileFactory
                                       config:
                                         threshold: 0.01
                                     DiscreteCollisionProfile2:
                                       class: DiscreteCollisionProfileFactory
                                       config:
                                         threshold: 0.01
                                   TrajOptCollisionConstraint:
                                     DiscreteCollisionProfile:
                                       class: DiscreteCollisionProfileFactory
                                       config:
                                         threshold: 0.01
                                     DiscreteCollisionProfile2:
                                       class: DiscreteCollisionProfileFactory
                                       config:
                                         threshold: 0.01)";

  {  // missing entry
    YAML::Node config = tesseract_common::loadYamlString(yaml_string, locator);
    auto plugin = config["profile_plugins"]["profiles"]["TrajOptCollisionCost"];
    plugin.remove("DiscreteCollisionProfile2");

    ProfilePluginFactory factory(config, locator);
    auto inv_kin = factory.create("TrajOptCollisionCost", "DiscreteCollisionProfile2");
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing class
    YAML::Node config = tesseract_common::loadYamlString(yaml_string, locator);
    auto plugin = config["profile_plugins"]["profiles"]["TrajOptCollisionCost"]["DiscreteCollisionProfile2"];
    plugin.remove("class");

    EXPECT_ANY_THROW(ProfilePluginFactory factory(config, locator));  // NOLINT
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
