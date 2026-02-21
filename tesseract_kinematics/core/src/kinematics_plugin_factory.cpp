/**
 * @file kinematics_factory.h
 * @brief Kinematics Factory
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <console_bridge/console.h>
#include <fstream>

static const std::string TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV = "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES";
static const std::string TESSERACT_KINEMATICS_PLUGINS_ENV = "TESSERACT_KINEMATICS_PLUGINS";

using tesseract::common::KinematicsPluginInfo;

namespace tesseract::kinematics
{
std::string InvKinFactory::getSection() { return "InvKin"; }

std::string FwdKinFactory::getSection() { return "FwdKin"; }

KinematicsPluginFactory::KinematicsPluginFactory()
{
  plugin_loader_.search_libraries_env = TESSERACT_KINEMATICS_PLUGINS_ENV;
  plugin_loader_.search_paths_env = TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_.search_paths.emplace_back(TESSERACT_KINEMATICS_PLUGIN_PATH);
  boost::split(
      plugin_loader_.search_libraries, TESSERACT_KINEMATICS_PLUGINS, boost::is_any_of(":"), boost::token_compress_on);

  tesseract::common::removeDuplicates(plugin_loader_.search_paths);
  tesseract::common::removeDuplicates(plugin_loader_.search_libraries);
}

void KinematicsPluginFactory::loadConfig(const YAML::Node& config)
{
  if (const YAML::Node& plugin_info = config[KinematicsPluginInfo::CONFIG_KEY])
  {
    auto kin_plugin_info = plugin_info.as<tesseract::common::KinematicsPluginInfo>();
    plugin_loader_.search_paths.insert(
        plugin_loader_.search_paths.end(), kin_plugin_info.search_paths.begin(), kin_plugin_info.search_paths.end());
    plugin_loader_.search_libraries.insert(plugin_loader_.search_libraries.end(),
                                           kin_plugin_info.search_libraries.begin(),
                                           kin_plugin_info.search_libraries.end());
    fwd_plugin_info_ = kin_plugin_info.fwd_plugin_infos;
    inv_plugin_info_ = kin_plugin_info.inv_plugin_infos;

    tesseract::common::removeDuplicates(plugin_loader_.search_paths);
    tesseract::common::removeDuplicates(plugin_loader_.search_libraries);
  }
}

KinematicsPluginFactory::KinematicsPluginFactory(YAML::Node config, const tesseract::common::ResourceLocator& locator)
  : KinematicsPluginFactory()
{
  tesseract::common::processYamlIncludeDirective(config, locator);
  loadConfig(config);
}

KinematicsPluginFactory::KinematicsPluginFactory(const std::filesystem::path& config,
                                                 const tesseract::common::ResourceLocator& locator)
  : KinematicsPluginFactory()
{
  loadConfig(tesseract::common::loadYamlFile(config.string(), locator));
}

KinematicsPluginFactory::KinematicsPluginFactory(const std::string& config,
                                                 const tesseract::common::ResourceLocator& locator)
  : KinematicsPluginFactory()
{
  loadConfig(tesseract::common::loadYamlString(config, locator));
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
KinematicsPluginFactory::~KinematicsPluginFactory() = default;

void KinematicsPluginFactory::addSearchPath(const std::string& path)
{
  auto& v = plugin_loader_.search_paths;
  if (std::find(v.begin(), v.end(), path) == v.end())
    v.push_back(path);
}

std::vector<std::string> KinematicsPluginFactory::getSearchPaths() const { return plugin_loader_.search_paths; }

void KinematicsPluginFactory::addSearchLibrary(const std::string& library_name)
{
  auto& v = plugin_loader_.search_libraries;
  if (std::find(v.begin(), v.end(), library_name) == v.end())
    v.push_back(library_name);
}

std::vector<std::string> KinematicsPluginFactory::getSearchLibraries() const { return plugin_loader_.search_libraries; }

void KinematicsPluginFactory::addFwdKinPlugin(const std::string& group_name,
                                              const std::string& solver_name,
                                              tesseract::common::PluginInfo plugin_info)
{
  fwd_plugin_info_[group_name].plugins[solver_name] = std::move(plugin_info);
}

std::map<std::string, tesseract::common::PluginInfoContainer> KinematicsPluginFactory::getFwdKinPlugins() const
{
  return fwd_plugin_info_;
}

void KinematicsPluginFactory::removeFwdKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.plugins.erase(solver_it);
  if (group_it->second.plugins.empty())
    fwd_plugin_info_.erase(group_it);

  if (group_it->second.default_plugin == solver_name)
    group_it->second.default_plugin.clear();
}

void KinematicsPluginFactory::setDefaultFwdKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default fwd kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default fwd kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.default_plugin = solver_name;
}

std::string KinematicsPluginFactory::getDefaultFwdKinPlugin(const std::string& group_name) const
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to get default fwd kin solver for a group '" + group_name +
                             "' that does not exist!");

  if (group_it->second.default_plugin.empty())
    return group_it->second.plugins.begin()->first;

  return group_it->second.default_plugin;
}

void KinematicsPluginFactory::addInvKinPlugin(const std::string& group_name,
                                              const std::string& solver_name,
                                              tesseract::common::PluginInfo plugin_info)
{
  inv_plugin_info_[group_name].plugins[solver_name] = std::move(plugin_info);
}

std::map<std::string, tesseract::common::PluginInfoContainer> KinematicsPluginFactory::getInvKinPlugins() const
{
  return inv_plugin_info_;
}

void KinematicsPluginFactory::removeInvKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.plugins.erase(solver_it);
  if (group_it->second.plugins.empty())
    inv_plugin_info_.erase(group_it);

  if (group_it->second.default_plugin == solver_name)
    group_it->second.default_plugin.clear();
}

void KinematicsPluginFactory::setDefaultInvKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default inv kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default inv kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.default_plugin = solver_name;
}

std::string KinematicsPluginFactory::getDefaultInvKinPlugin(const std::string& group_name) const
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to get default inv kin solver for a group '" + group_name +
                             "' that does not exist!");

  if (group_it->second.default_plugin.empty())
    return group_it->second.plugins.begin()->first;

  return group_it->second.default_plugin;
}

std::unique_ptr<ForwardKinematics>
KinematicsPluginFactory::createFwdKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract::scene_graph::SceneGraph& scene_graph,
                                      const tesseract::scene_graph::SceneState& scene_state) const
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get fwd kin solver '%s' for a group '%s' that does not "
                           "exist!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get fwd kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return createFwdKin(solver_name, solver_it->second, scene_graph, scene_state);
}

std::unique_ptr<ForwardKinematics>
KinematicsPluginFactory::createFwdKin(const std::string& solver_name,
                                      const tesseract::common::PluginInfo& plugin_info,
                                      const tesseract::scene_graph::SceneGraph& scene_graph,
                                      const tesseract::scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = fwd_kin_factories_.find(plugin_info.class_name);
    if (it != fwd_kin_factories_.end())
      return it->second->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_.createInstance<FwdKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    fwd_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

std::unique_ptr<InverseKinematics>
KinematicsPluginFactory::createInvKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract::scene_graph::SceneGraph& scene_graph,
                                      const tesseract::scene_graph::SceneState& scene_state) const
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get inv kin solver '%s' for a group '%s' that does not "
                           "exist!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  auto solver_it = group_it->second.plugins.find(solver_name);
  if (solver_it == group_it->second.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get inv kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return createInvKin(solver_name, solver_it->second, scene_graph, scene_state);
}

std::unique_ptr<InverseKinematics>
KinematicsPluginFactory::createInvKin(const std::string& solver_name,
                                      const tesseract::common::PluginInfo& plugin_info,
                                      const tesseract::scene_graph::SceneGraph& scene_graph,
                                      const tesseract::scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = inv_kin_factories_.find(plugin_info.class_name);
    if (it != inv_kin_factories_.end())
      return it->second->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_.createInstance<InvKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    inv_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception& ex)
  {
    CONSOLE_BRIDGE_logWarn(ex.what());
    return nullptr;
  }
}

void KinematicsPluginFactory::saveConfig(const std::filesystem::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node KinematicsPluginFactory::getConfig() const
{
  tesseract::common::KinematicsPluginInfo kinematic_plugins;
  kinematic_plugins.search_paths = plugin_loader_.search_paths;
  kinematic_plugins.search_libraries = plugin_loader_.search_libraries;
  kinematic_plugins.fwd_plugin_infos = fwd_plugin_info_;
  kinematic_plugins.inv_plugin_infos = inv_plugin_info_;

  YAML::Node config;
  config[KinematicsPluginInfo::CONFIG_KEY] = kinematic_plugins;

  return config;
}

}  // namespace tesseract::kinematics
