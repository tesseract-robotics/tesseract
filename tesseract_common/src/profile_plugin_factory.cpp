/**
 * @file profile_plugin_factory.cpp
 * @brief A plugin factory for producing a profiles
 *
 * @author Levi Armstrong
 * @date August 27, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#include <yaml-cpp/yaml.h>
#include <utility>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/profile.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>

static const std::string PLUGIN_DIRECTORIES_ENV = "TESSERACT_PROFILES_PLUGIN_DIRECTORIES";
static const std::string PLUGIN_ENV = "TESSERACT_PROFILES_PLUGINS";

namespace tesseract_common
{
std::string ProfileFactory::getSection() { return "Profile"; }

struct ProfilePluginFactory::Implementation
{
  std::shared_ptr<const ProfileFactoryData> factory_data;
  mutable std::map<std::string, ProfileFactory::Ptr> factories;
  std::map<std::string, tesseract_common::PluginInfoMap> plugin_infos;
  boost_plugin_loader::PluginLoader plugin_loader;
};

ProfilePluginFactory::ProfilePluginFactory() : impl_(std::make_unique<Implementation>())
{
  impl_->plugin_loader.search_libraries_env = PLUGIN_ENV;
  impl_->plugin_loader.search_paths_env = PLUGIN_DIRECTORIES_ENV;
}

ProfilePluginFactory::ProfilePluginFactory(const tesseract_common::ProfilesPluginInfo& config) : ProfilePluginFactory()
{
  loadConfig(config);
}

ProfilePluginFactory::ProfilePluginFactory(const YAML::Node& config, const tesseract_common::ResourceLocator& locator)
  : ProfilePluginFactory()
{
  loadConfig(config, locator);
}

ProfilePluginFactory::ProfilePluginFactory(const std::filesystem::path& config,
                                           const tesseract_common::ResourceLocator& locator)
  : ProfilePluginFactory()
{
  loadConfig(config, locator);
}

ProfilePluginFactory::ProfilePluginFactory(const std::string& config, const tesseract_common::ResourceLocator& locator)
  : ProfilePluginFactory()
{
  loadConfig(config, locator);
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
ProfilePluginFactory::~ProfilePluginFactory() = default;
ProfilePluginFactory::ProfilePluginFactory(ProfilePluginFactory&&) noexcept = default;
ProfilePluginFactory& ProfilePluginFactory::operator=(ProfilePluginFactory&&) noexcept = default;

void ProfilePluginFactory::setFactoryData(std::shared_ptr<const ProfileFactoryData> data)
{
  impl_->factory_data = std::move(data);
}

std::shared_ptr<const ProfileFactoryData> ProfilePluginFactory::getFactoryData() const
{
  return std::as_const(*impl_).factory_data;
}

void ProfilePluginFactory::loadConfig(const tesseract_common::ProfilesPluginInfo& config)
{
  impl_->plugin_loader.search_libraries.insert(
      impl_->plugin_loader.search_libraries.end(), config.search_libraries.begin(), config.search_libraries.end());
  impl_->plugin_loader.search_paths.insert(
      impl_->plugin_loader.search_paths.end(), config.search_paths.begin(), config.search_paths.end());
  impl_->plugin_infos.insert(config.plugin_infos.begin(), config.plugin_infos.end());
}

void ProfilePluginFactory::loadConfig(YAML::Node config)
{
  if (const YAML::Node& plugin_info = config[tesseract_common::ProfilesPluginInfo::CONFIG_KEY])
    loadConfig(plugin_info.as<tesseract_common::ProfilesPluginInfo>());
}

void ProfilePluginFactory::loadConfig(YAML::Node config, const tesseract_common::ResourceLocator& locator)
{
  tesseract_common::processYamlIncludeDirective(config, locator);
  loadConfig(config);
}

void ProfilePluginFactory::loadConfig(const std::filesystem::path& config,
                                      const tesseract_common::ResourceLocator& locator)
{
  loadConfig(tesseract_common::loadYamlFile(config.string(), locator));
}

void ProfilePluginFactory::loadConfig(const std::string& config, const tesseract_common::ResourceLocator& locator)
{
  loadConfig(tesseract_common::loadYamlString(config, locator));
}

void ProfilePluginFactory::addSearchPath(const std::string& path) { impl_->plugin_loader.search_paths.push_back(path); }

std::vector<std::string> ProfilePluginFactory::getSearchPaths() const
{
  return std::as_const(*impl_).plugin_loader.search_paths;
}

void ProfilePluginFactory::clearSearchPaths() { impl_->plugin_loader.search_paths.clear(); }

void ProfilePluginFactory::addSearchLibrary(const std::string& library_name)
{
  impl_->plugin_loader.search_libraries.push_back(library_name);
}

std::vector<std::string> ProfilePluginFactory::getSearchLibraries() const
{
  return std::as_const(*impl_).plugin_loader.search_libraries;
}

void ProfilePluginFactory::clearSearchLibraries() { impl_->plugin_loader.search_libraries.clear(); }

void ProfilePluginFactory::addPlugin(const std::string& ns,
                                     const std::string& name,
                                     tesseract_common::PluginInfo plugin_info)
{
  impl_->plugin_infos[ns][name] = std::move(plugin_info);
}

bool ProfilePluginFactory::hasPlugins() const { return !std::as_const(*impl_).plugin_infos.empty(); }

std::map<std::string, tesseract_common::PluginInfoMap> ProfilePluginFactory::getPlugins() const
{
  return std::as_const(*impl_).plugin_infos;
}

void ProfilePluginFactory::removePlugin(const std::string& ns, const std::string& name)
{
  auto ns_it = impl_->plugin_infos.find(ns);
  if (ns_it == impl_->plugin_infos.end())
    throw std::runtime_error("ProfilePluginFactory, tried to remove profile '" + name + "' for namespace '" + ns +
                             "' that does not exist!");

  auto profile_it = ns_it->second.find(name);
  if (profile_it == ns_it->second.end())
    throw std::runtime_error("ProfilePluginFactory, tried to remove profile '" + name +
                             "' that does not exist for namespace '" + ns + "'!");

  ns_it->second.erase(profile_it);
  if (ns_it->second.empty())
    impl_->plugin_infos.erase(ns_it);
}

std::unique_ptr<Profile> ProfilePluginFactory::create(const std::string& ns, const std::string& name) const
{
  auto ns_it = std::as_const(*impl_).plugin_infos.find(ns);
  if (ns_it == std::as_const(*impl_).plugin_infos.end())
  {
    CONSOLE_BRIDGE_logWarn("ProfilePluginFactory, tried to get profile '%s' for namespace '%s' that does not "
                           "exist!",
                           name.c_str(),
                           ns.c_str());
    return nullptr;
  }

  auto profile_it = ns_it->second.find(name);
  if (profile_it == ns_it->second.end())
  {
    CONSOLE_BRIDGE_logWarn("ProfilePluginFactory, tried to get profile '%s' that does not exist for namespace "
                           "'%s'!",
                           name.c_str(),
                           ns.c_str());
    return nullptr;
  }

  return create(name, profile_it->second);
}

std::unique_ptr<Profile> ProfilePluginFactory::create(const std::string& name,
                                                      const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto& factories = std::as_const(*impl_).factories;
    auto it = factories.find(plugin_info.class_name);
    if (it != factories.end())
      return it->second->create(name, plugin_info.config, impl_->factory_data, *this);

    auto plugin = std::as_const(*impl_).plugin_loader.createInstance<ProfileFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    factories[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config, impl_->factory_data, *this);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s', Details: %s", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

void ProfilePluginFactory::saveConfig(const std::filesystem::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node ProfilePluginFactory::getConfig() const
{
  tesseract_common::ProfilesPluginInfo plugins;
  plugins.search_paths = impl_->plugin_loader.search_paths;
  plugins.search_libraries = impl_->plugin_loader.search_libraries;
  plugins.plugin_infos = impl_->plugin_infos;

  YAML::Node config;
  config[tesseract_common::ProfilesPluginInfo::CONFIG_KEY] = plugins;

  return config;
}

ProfileDictionary ProfilePluginFactory::getProfileDictionary() const
{
  ProfileDictionary profiles;
  for (const auto& ns_pair : std::as_const(*impl_).plugin_infos)
  {
    for (const auto& profile_pair : ns_pair.second)
    {
      std::unique_ptr<Profile> profile = create(profile_pair.first, profile_pair.second);
      profiles.addProfile(ns_pair.first, profile_pair.first, std::move(profile));
    }
  }
  return profiles;
}
}  // namespace tesseract_common
