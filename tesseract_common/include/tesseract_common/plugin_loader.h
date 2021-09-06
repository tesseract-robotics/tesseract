/**
 * @file plugin_loader.h
 * @brief Plugin Loader to be used throughout Tesseract for loading plugins
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
#ifndef TESSERACT_COMMON_PLUGIN_LOADER_H
#define TESSERACT_COMMON_PLUGIN_LOADER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <set>
#include <unordered_map>
#include <string>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief This is a utility class for loading plugins within Tesseract
 * @details The library_name should not include the prefix 'lib' or suffix '.so'. It will add the correct prefix and
 * suffix based on the OS.
 *
 * It supports providing additional search paths and set environment variable which should be used when searching for
 * plugins.
 *
 * The plugin must be exported using the macro TESSERACT_ADD_PLUGIN.
 * In the example below, the first parameter is the derived object and the second is the assinged symbol name which is
 * used for looding Example: TESSERACT_ADD_PLUGIN(my_namespace::MyPlugin, plugin)
 *
 *   PluginLoader loader;
 *   loader.search_libraries.insert("my_plugin"); // libmy_plugin.so
 *   std::shared_ptr<PluginBase> p = loader.instantiate<PluginBase>("plugin");
 */
class PluginLoader
{
public:
  /** @brief Indicate is system folders may be search if plugin is not found in any of the paths */
  bool search_system_folders{ true };

  /** @brief A list of paths to search for plugins */
  std::set<std::string> search_paths;

  /** @brief A list of library names without the prefix or sufix that contain plugins*/
  std::set<std::string> search_libraries;

  /** @brief The environment variable containing plugin search paths */
  std::string search_paths_env;

  /**
   * @brief The environment variable containing plugins
   * @details The plugins are store ins the following formate.
   * The library name does not contain prefix or suffix
   *   Format: library_name:library_name1:library_name2
   */
  std::string search_libraries_env;

  /**
   * @brief Instantiate a plugin with the provided name
   * @param plugin_name The plugin name to find
   * @return A instantiation of the plugin, if nullptr it failed to create plugin
   */
  template <class PluginBase>
  std::shared_ptr<PluginBase> instantiate(const std::string& plugin_name) const;

  /**
   * @brief Check if plugin is available
   * @param plugin_name The plugin name to find
   * @return True if plugin is found
   */
  inline bool isPluginAvailable(const std::string& plugin_name) const;

  /**
   * @brief The number of plugins stored. The size of plugins variable
   * @return The number of plugins.
   */
  inline int count() const;
};

}  // namespace tesseract_common

#include <tesseract_common/plugin_loader.hpp>

#endif  // TESSERACT_COMMON_PLUGIN_LOADER_H
