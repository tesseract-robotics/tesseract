/**
 * @file plugin_loader.hpp
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
#ifndef TESSERACT_COMMON_PLUGIN_LOADER_HPP
#define TESSERACT_COMMON_PLUGIN_LOADER_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_loader.h>
#include <tesseract_common/class_loader.h>

namespace tesseract_common
{
inline std::list<std::string> parseEnvironmentVariableList(const std::string& env_variable)
{
  std::list<std::string> list;
  std::string evn_str = std::string(std::getenv(env_variable.c_str()));
  boost::split(list, evn_str, boost::is_any_of(":"), boost::token_compress_on);
  return list;
}

inline std::list<std::string> getAllSearchPaths(const std::string& search_paths_env,
                                                const std::list<std::string>& existing_search_paths)
{
  // Check for environment variable to override default library
  if (!search_paths_env.empty())
  {
    std::list<std::string> search_paths = parseEnvironmentVariableList(search_paths_env);
    search_paths.insert(search_paths.end(), existing_search_paths.begin(), existing_search_paths.end());
    return search_paths;
  }

  return existing_search_paths;
}

inline std::unordered_map<std::string, std::string> parseEnvironmentVariableMap(const std::string& env_variable)
{
  std::unordered_map<std::string, std::string> plugins;

  // Parse list first
  std::list<std::string> list;
  std::string evn_str = std::string(std::getenv(env_variable.c_str()));
  boost::split(list, evn_str, boost::is_any_of(":"), boost::token_compress_on);

  // Parse pairs
  for (const auto& pair : list)
  {
    std::vector<std::string> plugin_pair;
    boost::split(plugin_pair, pair, boost::is_any_of(","), boost::token_compress_on);
    if (plugin_pair.size() == 2)
    {
      plugins[plugin_pair[0]] = plugin_pair[1];
    }
    else
    {
      CONSOLE_BRIDGE_logWarn("Failed to parse information for plugin from environment variable: %s", pair.c_str());
    }
  }
  return plugins;
}

inline std::unordered_map<std::string, std::string>
getAllPlugins(const std::string& plugins_env, const std::unordered_map<std::string, std::string>& existing_plugins)
{
  // Check for environment variable to override default library
  if (!plugins_env.empty())
  {
    std::unordered_map<std::string, std::string> plugins = parseEnvironmentVariableMap(plugins_env);
    plugins.insert(existing_plugins.begin(), existing_plugins.end());
    return plugins;
  }

  return existing_plugins;
}

template <class PluginBase>
std::shared_ptr<PluginBase> PluginLoader::instantiate(const std::string& plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::unordered_map<std::string, std::string> plugins_local = getAllPlugins(plugins_env, plugins);
  auto it = plugins_local.find(plugin_name);
  if (it == plugins_local.end())
  {
    CONSOLE_BRIDGE_logError("Failed to find information for plugin: %s", plugin_name.c_str());
    return nullptr;
  }

  // Check for environment variable for search paths
  std::list<std::string> search_paths_local = getAllSearchPaths(search_paths_env, search_paths);
  for (const auto& path : search_paths_local)
  {
    if (ClassLoader::isClassAvailable(plugin_name, it->second, path))
      return ClassLoader::createSharedInstance<PluginBase>(plugin_name, it->second, path);
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders)
  {
    if (ClassLoader::isClassAvailable(plugin_name, it->second))
      return ClassLoader::createSharedInstance<PluginBase>(plugin_name, it->second);
  }

  CONSOLE_BRIDGE_logError("Failed to instantiate plugin '%s' from library: %s",
                          plugin_name.c_str(),
                          ClassLoader::decorate(it->second).c_str());
  return nullptr;
}

bool PluginLoader::isPluginAvailable(const std::string& plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::unordered_map<std::string, std::string> plugins_local = getAllPlugins(plugins_env, plugins);
  auto it = plugins_local.find(plugin_name);
  if (it == plugins_local.end())
    return false;

  // Check for environment variable to override default library
  std::list<std::string> search_paths_local = getAllSearchPaths(search_paths_env, search_paths);
  for (const auto& path : search_paths_local)
  {
    if (ClassLoader::isClassAvailable(plugin_name, it->second, path))
      return true;
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders)
    return ClassLoader::isClassAvailable(plugin_name, it->second);

  return false;
}

int PluginLoader::count() const { return static_cast<int>(getAllPlugins(plugins_env, plugins).size()); }

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PLUGIN_LOADER_HPP
