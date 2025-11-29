/**
 * @file contact_managers_plugin_factory.cpp
 * @brief Factory for loading contact managers as plugins
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>
#include <tesseract_collision/core/contact_managers_plugin_factory.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <console_bridge/console.h>

static const std::string TESSERACT_CONTACT_MANAGERS_PLUGIN_DIRECTORIES_ENV = "TESSERACT_CONTACT_MANAGERS_PLUGIN_"
                                                                             "DIRECTORIES";
static const std::string TESSERACT_CONTACT_MANAGERS_PLUGINS_ENV = "TESSERACT_CONTACT_MANAGERS_PLUGINS";

using tesseract_common::ContactManagersPluginInfo;

namespace tesseract_collision
{
std::string DiscreteContactManagerFactory::getSection() { return "DiscColl"; }

std::string ContinuousContactManagerFactory::getSection() { return "ContColl"; }

ContactManagersPluginFactory::ContactManagersPluginFactory()
{
  plugin_loader_.search_libraries_env = TESSERACT_CONTACT_MANAGERS_PLUGINS_ENV;
  plugin_loader_.search_paths_env = TESSERACT_CONTACT_MANAGERS_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_.search_paths.emplace_back(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH);
  boost::split(plugin_loader_.search_libraries,
               TESSERACT_CONTACT_MANAGERS_PLUGINS,
               boost::is_any_of(":"),
               boost::token_compress_on);

  tesseract_common::removeDuplicates(plugin_loader_.search_paths);
  tesseract_common::removeDuplicates(plugin_loader_.search_libraries);
}

void ContactManagersPluginFactory::loadConfig(const YAML::Node& config)
{
  if (const YAML::Node& plugin_info = config[ContactManagersPluginInfo::CONFIG_KEY])
  {
    auto cm_plugin_info = plugin_info.as<tesseract_common::ContactManagersPluginInfo>();
    plugin_loader_.search_paths.insert(
        plugin_loader_.search_paths.end(), cm_plugin_info.search_paths.begin(), cm_plugin_info.search_paths.end());
    plugin_loader_.search_libraries.insert(plugin_loader_.search_libraries.end(),
                                           cm_plugin_info.search_libraries.begin(),
                                           cm_plugin_info.search_libraries.end());
    discrete_plugin_info_ = cm_plugin_info.discrete_plugin_infos;
    continuous_plugin_info_ = cm_plugin_info.continuous_plugin_infos;

    tesseract_common::removeDuplicates(plugin_loader_.search_paths);
    tesseract_common::removeDuplicates(plugin_loader_.search_libraries);
  }
}

ContactManagersPluginFactory::ContactManagersPluginFactory(YAML::Node config,
                                                           const tesseract_common::ResourceLocator& locator)
  : ContactManagersPluginFactory()
{
  tesseract_common::processYamlIncludeDirective(config, locator);
  loadConfig(config);
}

ContactManagersPluginFactory::ContactManagersPluginFactory(const std::filesystem::path& config,
                                                           const tesseract_common::ResourceLocator& locator)
  : ContactManagersPluginFactory()
{
  loadConfig(tesseract_common::loadYamlFile(config.string(), locator));
}

ContactManagersPluginFactory::ContactManagersPluginFactory(const std::string& config,
                                                           const tesseract_common::ResourceLocator& locator)
  : ContactManagersPluginFactory()
{
  loadConfig(tesseract_common::loadYamlString(config, locator));
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
ContactManagersPluginFactory::~ContactManagersPluginFactory() = default;

void ContactManagersPluginFactory::addSearchPath(const std::string& path)
{
  auto& v = plugin_loader_.search_paths;
  if (std::find(v.begin(), v.end(), path) == v.end())
    v.push_back(path);
}

std::vector<std::string> ContactManagersPluginFactory::getSearchPaths() const { return plugin_loader_.search_paths; }

void ContactManagersPluginFactory::clearSearchPaths() { plugin_loader_.search_paths.clear(); }

void ContactManagersPluginFactory::addSearchLibrary(const std::string& library_name)
{
  auto& v = plugin_loader_.search_libraries;
  if (std::find(v.begin(), v.end(), library_name) == v.end())
    v.push_back(library_name);
}

std::vector<std::string> ContactManagersPluginFactory::getSearchLibraries() const
{
  return plugin_loader_.search_libraries;
}

void ContactManagersPluginFactory::clearSearchLibraries() { plugin_loader_.search_libraries.clear(); }

void ContactManagersPluginFactory::addDiscreteContactManagerPlugin(const std::string& name,
                                                                   tesseract_common::PluginInfo plugin_info)
{
  discrete_plugin_info_.plugins[name] = std::move(plugin_info);
}

bool ContactManagersPluginFactory::hasDiscreteContactManagerPlugins() const
{
  return !discrete_plugin_info_.plugins.empty();
}

tesseract_common::PluginInfoMap ContactManagersPluginFactory::getDiscreteContactManagerPlugins() const
{
  return discrete_plugin_info_.plugins;
}

void ContactManagersPluginFactory::removeDiscreteContactManagerPlugin(const std::string& name)
{
  auto cm_it = discrete_plugin_info_.plugins.find(name);
  if (cm_it == discrete_plugin_info_.plugins.end())
    throw std::runtime_error("ContactManagersPluginFactory, tried to remove discrete contact manager '" + name +
                             "' that does not exist!");

  discrete_plugin_info_.plugins.erase(cm_it);

  if (discrete_plugin_info_.default_plugin == name)
    discrete_plugin_info_.default_plugin.clear();
}

void ContactManagersPluginFactory::setDefaultDiscreteContactManagerPlugin(const std::string& name)
{
  auto cm_it = discrete_plugin_info_.plugins.find(name);
  if (cm_it == discrete_plugin_info_.plugins.end())
    throw std::runtime_error("ContactManagersPluginFactory, tried to set default discrete contact manager '" + name +
                             "' that does not exist!");

  discrete_plugin_info_.default_plugin = name;
}

std::string ContactManagersPluginFactory::getDefaultDiscreteContactManagerPlugin() const
{
  if (discrete_plugin_info_.plugins.empty())
    throw std::runtime_error("ContactManagersPluginFactory, tried to get default discrete contact manager but none "
                             "exist!");

  if (discrete_plugin_info_.default_plugin.empty())
    return discrete_plugin_info_.plugins.begin()->first;

  return discrete_plugin_info_.default_plugin;
}

void ContactManagersPluginFactory::addContinuousContactManagerPlugin(const std::string& name,
                                                                     tesseract_common::PluginInfo plugin_info)
{
  continuous_plugin_info_.plugins[name] = std::move(plugin_info);
}

bool ContactManagersPluginFactory::hasContinuousContactManagerPlugins() const
{
  return !continuous_plugin_info_.plugins.empty();
}

tesseract_common::PluginInfoMap ContactManagersPluginFactory::getContinuousContactManagerPlugins() const
{
  return continuous_plugin_info_.plugins;
}

void ContactManagersPluginFactory::removeContinuousContactManagerPlugin(const std::string& name)
{
  auto cm_it = continuous_plugin_info_.plugins.find(name);
  if (cm_it == continuous_plugin_info_.plugins.end())
    throw std::runtime_error("ContactManagersPluginFactory, tried to remove continuous contact manager '" + name +
                             "' that does not exist!");

  continuous_plugin_info_.plugins.erase(cm_it);

  if (continuous_plugin_info_.default_plugin == name)
    continuous_plugin_info_.default_plugin.clear();
}

void ContactManagersPluginFactory::setDefaultContinuousContactManagerPlugin(const std::string& name)
{
  auto cm_it = continuous_plugin_info_.plugins.find(name);
  if (cm_it == continuous_plugin_info_.plugins.end())
    throw std::runtime_error("ContactManagersPluginFactory, tried to set default continuous contact manager '" + name +
                             "' that does not exist!");

  continuous_plugin_info_.default_plugin = name;
}

std::string ContactManagersPluginFactory::getDefaultContinuousContactManagerPlugin() const
{
  if (continuous_plugin_info_.plugins.empty())
    throw std::runtime_error("ContactManagersPluginFactory, tried to get default continuous contact manager but none "
                             "exist!");

  if (continuous_plugin_info_.default_plugin.empty())
    return continuous_plugin_info_.plugins.begin()->first;

  return continuous_plugin_info_.default_plugin;
}

std::unique_ptr<DiscreteContactManager>
ContactManagersPluginFactory::createDiscreteContactManager(const std::string& name) const
{
  auto cm_it = discrete_plugin_info_.plugins.find(name);
  if (cm_it == discrete_plugin_info_.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("ContactManagersPluginFactory, tried to get discrete contact manager '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createDiscreteContactManager(name, cm_it->second);
}

std::unique_ptr<DiscreteContactManager>
ContactManagersPluginFactory::createDiscreteContactManager(const std::string& name,
                                                           const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto it = discrete_factories_.find(plugin_info.class_name);
    if (it != discrete_factories_.end())
      return it->second->create(name, plugin_info.config);

    auto plugin = plugin_loader_.createInstance<DiscreteContactManagerFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    discrete_factories_[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config);
  }
  // LCOV_EXCL_START
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
  // LCOV_EXCL_STOP
}

std::unique_ptr<ContinuousContactManager>
ContactManagersPluginFactory::createContinuousContactManager(const std::string& name) const
{
  auto cm_it = continuous_plugin_info_.plugins.find(name);
  if (cm_it == continuous_plugin_info_.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("ContactManagersPluginFactory, tried to get continuous contact manager '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createContinuousContactManager(name, cm_it->second);
}

std::unique_ptr<ContinuousContactManager>
ContactManagersPluginFactory::createContinuousContactManager(const std::string& name,
                                                             const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto it = continuous_factories_.find(plugin_info.class_name);
    if (it != continuous_factories_.end())
      return it->second->create(name, plugin_info.config);

    auto plugin = plugin_loader_.createInstance<ContinuousContactManagerFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    continuous_factories_[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config);
  }
  // LCOV_EXCL_START
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
  // LCOV_EXCL_STOP
}

void ContactManagersPluginFactory::saveConfig(const std::filesystem::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node ContactManagersPluginFactory::getConfig() const
{
  tesseract_common::ContactManagersPluginInfo cm_plugins;
  cm_plugins.search_paths = plugin_loader_.search_paths;
  cm_plugins.search_libraries = plugin_loader_.search_libraries;
  cm_plugins.discrete_plugin_infos = discrete_plugin_info_;
  cm_plugins.continuous_plugin_infos = continuous_plugin_info_;

  YAML::Node config;
  config[ContactManagersPluginInfo::CONFIG_KEY] = cm_plugins;

  return config;
}

}  // namespace tesseract_collision
