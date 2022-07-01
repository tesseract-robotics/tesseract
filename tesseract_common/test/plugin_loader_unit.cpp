/**
 * @file plugin_loader_unit.h
 * @brief Plugin Loader Unit Tests
 *
 * @author Levi Armstrong
 * @date March 25, 2021
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/class_loader.h>
#include <tesseract_common/plugin_loader.h>
#include "test_plugin_base.h"

const std::string tesseract_common::TestPluginBase::SECTION_NAME = "TestBase";

TEST(TesseractClassLoaderUnit, parseEnvironmentVariableListUnit)  // NOLINT
{
#ifndef _WIN32
  std::string env_var = "UNITTESTENV=a:b:c";
#else
  std::string env_var = "UNITTESTENV=a;b;c";
#endif
  putenv(env_var.data());
  std::set<std::string> s = tesseract_common::parseEnvironmentVariableList("UNITTESTENV");
  std::vector<std::string> v(s.begin(), s.end());
  EXPECT_EQ(v[0], "a");
  EXPECT_EQ(v[1], "b");
  EXPECT_EQ(v[2], "c");
}

TEST(TesseractClassLoaderUnit, LoadTestPlugin)  // NOLINT
{
  using tesseract_common::ClassLoader;
  using tesseract_common::TestPluginBase;
  const std::string lib_name = "tesseract_common_test_plugin_multiply";
  const std::string lib_dir = std::string(TEST_PLUGIN_DIR);
  const std::string symbol_name = "plugin";

  {
    std::vector<std::string> sections = ClassLoader::getAvailableSections(lib_name, lib_dir);
    EXPECT_EQ(sections.size(), 1);
    EXPECT_EQ(sections.at(0), "TestBase");

    sections = ClassLoader::getAvailableSections(lib_name, lib_dir, true);
    EXPECT_TRUE(sections.size() > 1);
  }

  {
    std::vector<std::string> symbols = ClassLoader::getAvailableSymbols("TestBase", lib_name, lib_dir);
    EXPECT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), symbol_name);
  }

  {
    EXPECT_TRUE(ClassLoader::isClassAvailable(symbol_name, lib_name, lib_dir));
    auto plugin = ClassLoader::createSharedInstance<TestPluginBase>(symbol_name, lib_name, lib_dir);
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }

// For some reason on Ubuntu 18.04 it does not search the current directory when only the library name is provided
#if BOOST_VERSION > 106800
  {
    EXPECT_TRUE(ClassLoader::isClassAvailable(symbol_name, lib_name));
    auto plugin = ClassLoader::createSharedInstance<TestPluginBase>(symbol_name, lib_name);
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
#endif

  {
    EXPECT_FALSE(ClassLoader::isClassAvailable(symbol_name, lib_name, "does_not_exist"));
    EXPECT_FALSE(ClassLoader::isClassAvailable(symbol_name, "does_not_exist", lib_dir));
    EXPECT_FALSE(ClassLoader::isClassAvailable("does_not_exist", lib_name, lib_dir));
  }

  {
    EXPECT_FALSE(ClassLoader::isClassAvailable(symbol_name, "does_not_exist"));
    EXPECT_FALSE(ClassLoader::isClassAvailable("does_not_exist", lib_name));
  }

  {
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(symbol_name, lib_name, "does_not_exist"));
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(symbol_name, "does_not_exist", lib_dir));
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>("does_not_exist", lib_name, lib_dir));
  }

  {
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(symbol_name, "does_not_exist"));  // NOLINT
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>("does_not_exist", lib_name));     // NOLINT
  }
}

TEST(TesseractPluginLoaderUnit, LoadTestPlugin)  // NOLINT
{
  using tesseract_common::PluginLoader;
  using tesseract_common::TestPluginBase;

  {
    PluginLoader plugin_loader;
    plugin_loader.search_paths.insert(std::string(TEST_PLUGIN_DIR));
    plugin_loader.search_libraries.insert("tesseract_common_test_plugin_multiply");

    EXPECT_TRUE(plugin_loader.isPluginAvailable("plugin"));
    auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);

    std::vector<std::string> sections = plugin_loader.getAvailableSections();
    EXPECT_EQ(sections.size(), 1);
    EXPECT_EQ(sections.at(0), "TestBase");

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
#if BOOST_VERSION > 106800
  {
    PluginLoader plugin_loader;
    plugin_loader.search_libraries.insert("tesseract_common_test_plugin_multiply");

    EXPECT_TRUE(plugin_loader.isPluginAvailable("plugin"));
    auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
#endif

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_paths.insert("does_not_exist");
    plugin_loader.search_libraries.insert("tesseract_common_test_plugin_multiply");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.insert("tesseract_common_test_plugin_multiply");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("does_not_exist"));
      auto plugin = plugin_loader.instantiate<TestPluginBase>("does_not_exist");
      EXPECT_TRUE(plugin == nullptr);
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("does_not_exist"));
      auto plugin = plugin_loader.instantiate<TestPluginBase>("does_not_exist");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.insert("does_not_exist");

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
      auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
    auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
    EXPECT_TRUE(plugin == nullptr);
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    EXPECT_FALSE(plugin_loader.isPluginAvailable("plugin"));
    auto plugin = plugin_loader.instantiate<TestPluginBase>("plugin");
    EXPECT_TRUE(plugin == nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
