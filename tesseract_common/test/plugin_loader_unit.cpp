/**
 * @file plugin_loader_unit.h
 * @brief Plugin Loader Unit Tests
 *
 * @author Levi Armstrong
 * @date March 25, 2021
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost_plugin_loader/utils.h>
#include "test_plugin_base.h"

TEST(TesseractClassLoaderUnit, parseEnvironmentVariableListUnit)  // NOLINT
{
#ifndef _WIN32
  std::string env_var = "UNITTESTENV=c:b:a";
#else
  std::string env_var = "UNITTESTENV=c;b;a";
#endif
  putenv(env_var.data());
  std::vector<std::string> s = boost_plugin_loader::parseEnvironmentVariableList("UNITTESTENV");
  EXPECT_EQ(s[0], "c");
  EXPECT_EQ(s[1], "b");
  EXPECT_EQ(s[2], "a");
}

TEST(TesseractClassLoaderUnit, LoadTestPlugin)  // NOLINT
{
  using tesseract::common::TestPluginBase;
  const std::string lib_name = "tesseract_common_test_plugins";
  const std::string lib_dir = std::string(TEST_PLUGIN_DIR);
  const std::string symbol_name = "plugin";

  // Load the library
  const std::optional<boost::dll::shared_library> lib_opt =
      boost_plugin_loader::loadLibrary(boost::filesystem::path(lib_dir) / lib_name);
  EXPECT_TRUE(lib_opt.has_value());
  const boost::dll::shared_library& lib = lib_opt.value();  // NOLINT

  {
    std::vector<std::string> sections = boost_plugin_loader::getAllAvailableSections(lib);
    EXPECT_EQ(sections.size(), 2);
    EXPECT_EQ(sections.at(0), "TestBase");
    EXPECT_EQ(sections.at(1), "Profile");

    sections = boost_plugin_loader::getAllAvailableSections(lib, true);
    EXPECT_TRUE(sections.size() > 1);
  }

  {
    std::vector<std::string> symbols = boost_plugin_loader::getAllAvailableSymbols(lib, "TestBase");
    EXPECT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), symbol_name);
  }

  {
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(boost_plugin_loader::createSharedInstance<TestPluginBase>(lib, "does_not_exist"));
  }
}

TEST(TesseractPluginLoaderUnit, LoadTestPlugin)  // NOLINT
{
  using boost_plugin_loader::PluginLoader;
  using tesseract::common::TestPluginBase;

  {
    PluginLoader plugin_loader;
    plugin_loader.search_paths.emplace_back(TEST_PLUGIN_DIR);
    plugin_loader.search_libraries.emplace_back("tesseract_common_test_plugins");

    EXPECT_TRUE(plugin_loader.isPluginAvailable("plugin"));
    auto plugin = plugin_loader.createInstance<TestPluginBase>("plugin");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);

    std::vector<std::string> sections = plugin_loader.getAvailableSections();
    EXPECT_EQ(sections.size(), 2);
    EXPECT_EQ(sections.at(0), "TestBase");
    EXPECT_EQ(sections.at(1), "Profile");

    sections = plugin_loader.getAvailableSections(true);
    EXPECT_TRUE(sections.size() > 1);

    std::vector<std::string> symbols = plugin_loader.getAvailablePlugins<TestPluginBase>();
    EXPECT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), "plugin");

    symbols = plugin_loader.getAvailablePlugins("TestBase");
    EXPECT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), "plugin");
  }

  // For some reason on Ubuntu 18.04 it does not search the current directory when only the library name is provided
  // #if BOOST_VERSION > 106800 && !__APPLE__
  //   {
  //     PluginLoader plugin_loader;
  //     plugin_loader.search_paths.insert(".");
  //     plugin_loader.search_libraries.insert("tesseract_common_test_plugins");

  //     EXPECT_TRUE(plugin_loader.isPluginAvailable("plugin"));
  //     auto plugin = plugin_loader.createInstance<TestPluginBase>("plugin");
  //     EXPECT_TRUE(plugin != nullptr);
  //     EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  //   }
  // #endif

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_paths.emplace_back("does_not_exist");
    plugin_loader.search_libraries.emplace_back("tesseract_common_test_plugins");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("plugin"));  // NOLINT
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.emplace_back("tesseract_common_test_plugins");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("does_not_exist"));
      EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("does_not_exist"));  // NOLINT
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("does_not_exist"));
      EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("does_not_exist"));  // NOLINT
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.emplace_back("does_not_exist");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("plugin"));  // NOLINT
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("plugin"));  // NOLINT
    }
  }

  {
    PluginLoader plugin_loader;
    EXPECT_ANY_THROW(plugin_loader.isPluginAvailable("plugin"));               // NOLINT
    EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("plugin"));  // NOLINT
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    EXPECT_ANY_THROW(plugin_loader.isPluginAvailable("plugin"));               // NOLINT
    EXPECT_ANY_THROW(plugin_loader.createInstance<TestPluginBase>("plugin"));  // NOLINT
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
