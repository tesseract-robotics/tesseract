/**
 * @file contact_managers_factory_unit.cpp
 * @brief Tesseract collision contact managers factory test
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
#include <gtest/gtest.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/contact_managers_plugin_factory.h>

using namespace tesseract_collision;

void runContactManagersFactoryTest(const tesseract_common::fs::path& config_path)
{
  ContactManagersPluginFactory factory(config_path);
  YAML::Node plugin_config = YAML::LoadFile(config_path.string());

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::set<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::set<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  factory.saveConfig(tesseract_common::fs::path(tesseract_common::getTempPath()) / "contact_manager_plugins_export."
                                                                                   "yaml");

  // Failures
  {
    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract_common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract_common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadAndExportPluginTest)  // NOLINT
{
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path config_path = file_path.parent_path() / "contact_manager_plugins.yaml";
  runContactManagersFactoryTest(config_path);

  tesseract_common::fs::path export_config_path = tesseract_common::fs::path(tesseract_common::getTempPath()) / "contac"
                                                                                                                "t_"
                                                                                                                "manage"
                                                                                                                "r_"
                                                                                                                "plugin"
                                                                                                                "s_"
                                                                                                                "export"
                                                                                                                ".yaml";
  runContactManagersFactoryTest(export_config_path);
}

TEST(TesseractContactManagersFactoryUnit, LoadStringPluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
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
                                  class: BulletCastSimpleManagerFactory)";

  ContactManagersPluginFactory factory(config);
  YAML::Node plugin_config = YAML::Load(config);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::set<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::set<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, PluginFactorAPIUnit)  // NOLINT
{
  ContactManagersPluginFactory factory;
  EXPECT_FALSE(factory.getSearchPaths().empty());
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_FALSE(factory.getSearchLibraries().empty());
  EXPECT_EQ(factory.getSearchLibraries().size(), 2);
  EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 0);
  EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 0);
  EXPECT_ANY_THROW(factory.getDefaultDiscreteContactManagerPlugin());    // NOLINT
  EXPECT_ANY_THROW(factory.getDefaultContinuousContactManagerPlugin());  // NOLINT

  factory.addSearchPath("/usr/local/lib");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 2);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 3);

  {
    tesseract_common::PluginInfoMap map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract_common::PluginInfo pi;
    pi.class_name = "TestDiscreteManagerFactory";
    factory.addDiscreteContactManagerPlugin("TestDiscreteManager", pi);
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 1);

    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("TestDiscreteManager") != map.end());
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "TestDiscreteManager");

    tesseract_common::PluginInfo pi2;
    pi2.class_name = "Test2DiscreteManagerFactory";
    factory.addDiscreteContactManagerPlugin("Test2DiscreteManager", pi2);
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 2);

    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2DiscreteManager") != map.end());
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "Test2DiscreteManager");
    factory.setDefaultDiscreteContactManagerPlugin("TestDiscreteManager");
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "TestDiscreteManager");

    factory.removeDiscreteContactManagerPlugin("TestDiscreteManager");
    map = factory.getDiscreteContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2DiscreteManager") != map.end());
    EXPECT_EQ(factory.getDiscreteContactManagerPlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultDiscreteContactManagerPlugin(), "Test2DiscreteManager");

    // Failures
    EXPECT_ANY_THROW(factory.removeDiscreteContactManagerPlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultDiscreteContactManagerPlugin("DoesNotExist"));  // NOLINT
  }

  {
    tesseract_common::PluginInfoMap map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract_common::PluginInfo pi;
    pi.class_name = "TestContinuousManagerFactory";
    factory.addContinuousContactManagerPlugin("TestContinuousManager", pi);
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 1);

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("TestContinuousManager") != map.end());
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "TestContinuousManager");

    tesseract_common::PluginInfo pi2;
    pi2.class_name = "Test2ContinuousManagerFactory";
    factory.addContinuousContactManagerPlugin("Test2ContinuousManager", pi2);
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 2);

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2ContinuousManager") != map.end());
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "Test2ContinuousManager");
    factory.setDefaultContinuousContactManagerPlugin("TestContinuousManager");
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "TestContinuousManager");

    factory.removeContinuousContactManagerPlugin("TestContinuousManager");

    map = factory.getContinuousContactManagerPlugins();
    EXPECT_TRUE(map.find("Test2ContinuousManager") != map.end());
    EXPECT_EQ(factory.getContinuousContactManagerPlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultContinuousContactManagerPlugin(), "Test2ContinuousManager");

    // Failures
    EXPECT_ANY_THROW(factory.removeContinuousContactManagerPlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultContinuousContactManagerPlugin("DoesNotExist"));  // NOLINT
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadOnlyDiscretePluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
                                BulletDiscreteSimpleManager:
                                  class: BulletDiscreteSimpleManagerFactory
                                FCLDiscreteBVHManager:
                                  class: FCLDiscreteBVHManagerFactory)";

  ContactManagersPluginFactory factory(config);
  YAML::Node plugin_config = YAML::Load(config);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& discrete_plugins = plugin_info["discrete_plugins"]["plugins"];

  {
    std::set<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::set<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(discrete_plugins.size(), 3);
  for (auto cm_it = discrete_plugins.begin(); cm_it != discrete_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

TEST(TesseractContactManagersFactoryUnit, LoadOnlyContinuousPluginTest)  // NOLINT
{
  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories
                              - tesseract_collision_fcl_factories
                            continuous_plugins:
                              default: BulletCastBVHManager
                              plugins:
                                BulletCastBVHManager:
                                  class: BulletCastBVHManagerFactory
                                BulletCastSimpleManager:
                                  class: BulletCastSimpleManagerFactory)";

  ContactManagersPluginFactory factory(config);
  YAML::Node plugin_config = YAML::Load(config);

  const YAML::Node& plugin_info = plugin_config["contact_manager_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& continuous_plugins = plugin_info["continuous_plugins"]["plugins"];

  {
    std::set<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::set<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 2);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(continuous_plugins.size(), 2);
  for (auto cm_it = continuous_plugins.begin(); cm_it != continuous_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();

    ContinuousContactManager::UPtr cm = factory.createContinuousContactManager(name);
    EXPECT_TRUE(cm != nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
