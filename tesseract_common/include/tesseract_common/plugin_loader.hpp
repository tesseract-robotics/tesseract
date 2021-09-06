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
#include <ostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_loader.h>
#include <tesseract_common/class_loader.h>

namespace tesseract_common
{
inline std::set<std::string> parseEnvironmentVariableList(const std::string& env_variable)
{
  std::set<std::string> list;
  char* env_var = std::getenv(env_variable.c_str());
  if (env_var == nullptr)  // Environment variable not found
    return list;

  std::string evn_str = std::string(env_var);
  boost::split(list, evn_str, boost::is_any_of(":"), boost::token_compress_on);
  return list;
}

inline std::set<std::string> getAllSearchPaths(const std::string& search_paths_env,
                                               const std::set<std::string>& existing_search_paths)
{
  // Check for environment variable to override default library
  if (!search_paths_env.empty())
  {
    std::set<std::string> search_paths = parseEnvironmentVariableList(search_paths_env);
    search_paths.insert(existing_search_paths.begin(), existing_search_paths.end());
    return search_paths;
  }

  return existing_search_paths;
}

inline std::set<std::string> getAllSearchLibraries(const std::string& search_libraries_env,
                                                   const std::set<std::string>& existing_search_libraries)
{
  // Check for environment variable to override default library
  if (!search_libraries_env.empty())
  {
    std::set<std::string> search_libraries = parseEnvironmentVariableList(search_libraries_env);
    search_libraries.insert(existing_search_libraries.begin(), existing_search_libraries.end());
    return search_libraries;
  }

  return existing_search_libraries;
}

template <class PluginBase>
std::shared_ptr<PluginBase> PluginLoader::instantiate(const std::string& plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::set<std::string> plugins_local = getAllSearchLibraries(search_libraries_env, search_libraries);
  if (plugins_local.empty())
  {
    CONSOLE_BRIDGE_logError("No plugin libraries were provided!");
    return nullptr;
  }

  // Check for environment variable for search paths
  std::set<std::string> search_paths_local = getAllSearchPaths(search_paths_env, search_paths);
  for (const auto& path : search_paths_local)
  {
    for (const auto& library : search_libraries)
    {
      if (ClassLoader::isClassAvailable(plugin_name, library, path))
        return ClassLoader::createSharedInstance<PluginBase>(plugin_name, library, path);
    }
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders)
  {
    for (const auto& library : search_libraries)
    {
      if (ClassLoader::isClassAvailable(plugin_name, library))
        return ClassLoader::createSharedInstance<PluginBase>(plugin_name, library);
    }
  }

  std::stringstream msg;
  if (search_system_folders)
    msg << std::endl << "Search Paths (Search System Folders: True):" << std::endl;
  else
    msg << std::endl << "Search Paths (Search System Folders: False):" << std::endl;

  for (const auto& path : search_paths_local)
    msg << "    - " + path << std::endl;

  msg << "Search Libraries:" << std::endl;
  for (const auto& library : search_libraries)
    msg << "    - " + ClassLoader::decorate(library) << std::endl;

  CONSOLE_BRIDGE_logError("Failed to instantiate plugin '%s', Details: %s", plugin_name.c_str(), msg.str().c_str());

  return nullptr;
}

bool PluginLoader::isPluginAvailable(const std::string& plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::set<std::string> plugins_local = getAllSearchLibraries(search_libraries_env, search_libraries);
  if (plugins_local.empty())
  {
    CONSOLE_BRIDGE_logError("No plugin libraries were provided!");
    return false;
  }

  // Check for environment variable to override default library
  std::set<std::string> search_paths_local = getAllSearchPaths(search_paths_env, search_paths);
  for (const auto& path : search_paths_local)
  {
    for (const auto& library : search_libraries)
    {
      if (ClassLoader::isClassAvailable(plugin_name, library, path))
        return true;
    }
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders)
  {
    for (const auto& library : search_libraries)
    {
      if (ClassLoader::isClassAvailable(plugin_name, library))
        return true;
    }
  }

  return false;
}

int PluginLoader::count() const
{
  return static_cast<int>(getAllSearchLibraries(search_libraries_env, search_libraries).size());
}

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PLUGIN_LOADER_HPP
