/**
 * @file kinematics_factory.h
 * @brief Kinematics Factory
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/plugin_loader.hpp>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

const std::string TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV = "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES";
const std::string TESSERACT_KINEMATICS_PLUGINS_ENV = "TESSERACT_KINEMATICS_PLUGINS";

using tesseract_common::KinematicsPluginInfo;

namespace tesseract_kinematics
{
KinematicsPluginFactory::KinematicsPluginFactory()
{
  plugin_loader_.search_libraries_env = TESSERACT_KINEMATICS_PLUGINS_ENV;
  plugin_loader_.search_paths_env = TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_.search_paths.insert(TESSERACT_KINEMATICS_PLUGIN_PATH);
  boost::split(
      plugin_loader_.search_libraries, TESSERACT_KINEMATICS_PLUGINS, boost::is_any_of(":"), boost::token_compress_on);
}

KinematicsPluginFactory::KinematicsPluginFactory(YAML::Node config) : KinematicsPluginFactory()
{
  if (const YAML::Node& plugin_info = config[KinematicsPluginInfo::CONFIG_KEY])
  {
    auto kin_plugin_info = plugin_info.as<tesseract_common::KinematicsPluginInfo>();
    plugin_loader_.search_paths.insert(kin_plugin_info.search_paths.begin(), kin_plugin_info.search_paths.end());
    plugin_loader_.search_libraries.insert(kin_plugin_info.search_libraries.begin(),
                                           kin_plugin_info.search_libraries.end());
    fwd_plugin_info_ = kin_plugin_info.fwd_plugin_infos;
    inv_plugin_info_ = kin_plugin_info.inv_plugin_infos;
  }
}

KinematicsPluginFactory::KinematicsPluginFactory(const tesseract_common::fs::path& config)
  : KinematicsPluginFactory(YAML::LoadFile(config.string()))
{
}

KinematicsPluginFactory::KinematicsPluginFactory(const std::string& config)
  : KinematicsPluginFactory(YAML::Load(config))
{
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
KinematicsPluginFactory::~KinematicsPluginFactory() = default;

void KinematicsPluginFactory::addSearchPath(const std::string& path) { plugin_loader_.search_paths.insert(path); }

std::set<std::string> KinematicsPluginFactory::getSearchPaths() const { return plugin_loader_.search_paths; }

void KinematicsPluginFactory::addSearchLibrary(const std::string& library_name)
{
  plugin_loader_.search_libraries.insert(library_name);
}

std::set<std::string> KinematicsPluginFactory::getSearchLibraries() const { return plugin_loader_.search_libraries; }

void KinematicsPluginFactory::addFwdKinPlugin(const std::string& group_name,
                                              const std::string& solver_name,
                                              tesseract_common::PluginInfo plugin_info)
{
  fwd_plugin_info_[group_name].plugins[solver_name] = std::move(plugin_info);
}

std::map<std::string, tesseract_common::PluginInfoContainer> KinematicsPluginFactory::getFwdKinPlugins() const
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
                                              tesseract_common::PluginInfo plugin_info)
{
  inv_plugin_info_[group_name].plugins[solver_name] = std::move(plugin_info);
}

std::map<std::string, tesseract_common::PluginInfoContainer> KinematicsPluginFactory::getInvKinPlugins() const
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

ForwardKinematics::UPtr
KinematicsPluginFactory::createFwdKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
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

ForwardKinematics::UPtr
KinematicsPluginFactory::createFwdKin(const std::string& solver_name,
                                      const tesseract_common::PluginInfo& plugin_info,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = fwd_kin_factories_.find(plugin_info.class_name);
    if (it != fwd_kin_factories_.end())
      return it->second->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_.instantiate<FwdKinFactory>(plugin_info.class_name);
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

InverseKinematics::UPtr
KinematicsPluginFactory::createInvKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
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

InverseKinematics::UPtr
KinematicsPluginFactory::createInvKin(const std::string& solver_name,
                                      const tesseract_common::PluginInfo& plugin_info,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = inv_kin_factories_.find(plugin_info.class_name);
    if (it != inv_kin_factories_.end())
      return it->second->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_.instantiate<InvKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    inv_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(solver_name, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

void KinematicsPluginFactory::saveConfig(const tesseract_common::fs::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node KinematicsPluginFactory::getConfig() const
{
  tesseract_common::KinematicsPluginInfo kinematic_plugins;
  kinematic_plugins.search_paths = plugin_loader_.search_paths;
  kinematic_plugins.search_libraries = plugin_loader_.search_libraries;
  kinematic_plugins.fwd_plugin_infos = fwd_plugin_info_;
  kinematic_plugins.inv_plugin_infos = inv_plugin_info_;

  YAML::Node config;
  config[KinematicsPluginInfo::CONFIG_KEY] = kinematic_plugins;

  return config;
}

}  // namespace tesseract_kinematics
