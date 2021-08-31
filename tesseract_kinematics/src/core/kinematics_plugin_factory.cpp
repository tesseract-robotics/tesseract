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
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

const std::string TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV = "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES";
const std::string TESSERACT_KINEMATICS_PLUGINS_ENV = "TESSERACT_KINEMATICS_PLUGINS";

namespace tesseract_kinematics
{
KinematicsPluginFactory::KinematicsPluginFactory() : plugin_loader_(std::make_unique<tesseract_common::PluginLoader>())
{
  plugin_loader_->plugins_env = TESSERACT_KINEMATICS_PLUGINS_ENV;
  plugin_loader_->search_paths_env = TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_->search_paths.insert(TESSERACT_KINEMATICS_PLUGIN_PATH);
  boost::split(plugin_loader_->plugins, TESSERACT_KINEMATICS_PLUGINS, boost::is_any_of(":"), boost::token_compress_on);
}

KinematicsPluginFactory::KinematicsPluginFactory(YAML::Node config) : KinematicsPluginFactory()
{
  if (const YAML::Node& plugin_info = config["kinematic_plugins"])
  {
    if (const YAML::Node& search_paths = plugin_info["search_paths"])
    {
      for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
        addSearchPath(it->as<std::string>());
    }

    if (const YAML::Node& search_libraries = plugin_info["search_libraries"])
    {
      for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
        addSearchLibrary(it->as<std::string>());
    }

    if (const YAML::Node& fwd_kin_plugins = plugin_info["fwd_kin_plugins"])
    {
      for (auto it = fwd_kin_plugins.begin(); it != fwd_kin_plugins.end(); ++it)
      {
        KinematicsPluginInfo info;
        const YAML::Node& plugin = *it;
        info.name = plugin["name"].as<std::string>();
        info.class_name = plugin["class"].as<std::string>();
        info.group = plugin["group"].as<std::string>();
        if (plugin["config"])
          info.config = plugin["config"];

        addFwdKinPlugin(info);
      }
    }

    if (const YAML::Node& inv_kin_plugins = plugin_info["inv_kin_plugins"])
    {
      for (auto it = inv_kin_plugins.begin(); it != inv_kin_plugins.end(); ++it)
      {
        KinematicsPluginInfo info;
        const YAML::Node& plugin = *it;
        info.name = plugin["name"].as<std::string>();
        info.class_name = plugin["class"].as<std::string>();
        info.group = plugin["group"].as<std::string>();
        if (plugin["config"])
          info.config = plugin["config"];

        addInvKinPlugin(info);
      }
    }
  }
}

KinematicsPluginFactory::KinematicsPluginFactory(boost::filesystem::path config)
  : KinematicsPluginFactory(YAML::LoadFile(config.string()))
{
}

KinematicsPluginFactory::KinematicsPluginFactory(std::string config) : KinematicsPluginFactory(YAML::Load(config)) {}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
KinematicsPluginFactory::~KinematicsPluginFactory() = default;

void KinematicsPluginFactory::addSearchPath(const std::string& path) { plugin_loader_->search_paths.insert(path); }

const std::set<std::string>& KinematicsPluginFactory::getSearchPaths() const { return plugin_loader_->search_paths; }

void KinematicsPluginFactory::addSearchLibrary(const std::string& library_name)
{
  plugin_loader_->plugins.insert(library_name);
}

const std::set<std::string>& KinematicsPluginFactory::getSearchLibraries() const { return plugin_loader_->plugins; }

void KinematicsPluginFactory::addFwdKinPlugin(KinematicsPluginInfo plugin_info)
{
  fwd_plugin_info_[plugin_info.group][plugin_info.name] = plugin_info;
}

void KinematicsPluginFactory::removeFwdKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.erase(solver_it);
  if (group_it->second.empty())
    fwd_plugin_info_.erase(group_it);
}

void KinematicsPluginFactory::addInvKinPlugin(KinematicsPluginInfo plugin_info)
{
  inv_plugin_info_[plugin_info.group][plugin_info.name] = plugin_info;
}

void KinematicsPluginFactory::removeInvKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.erase(solver_it);
  if (group_it->second.empty())
    fwd_plugin_info_.erase(group_it);
}

ForwardKinematics::UPtr KinematicsPluginFactory::getFwdKin(const std::string& group_name,
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

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get fwd kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return getFwdKin(solver_it->second, scene_graph, scene_state);
}

ForwardKinematics::UPtr KinematicsPluginFactory::getFwdKin(const KinematicsPluginInfo& plugin_info,
                                                           const tesseract_scene_graph::SceneGraph& scene_graph,
                                                           const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = fwd_kin_factories_.find(plugin_info.class_name);
    if (it != fwd_kin_factories_.end())
      return it->second->create(plugin_info.group, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_->instantiate<FwdKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    fwd_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(plugin_info.group, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

InverseKinematics::UPtr KinematicsPluginFactory::getInvKin(const std::string& group_name,
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

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get inv kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return getInvKin(solver_it->second, scene_graph, scene_state);
}

InverseKinematics::UPtr KinematicsPluginFactory::getInvKin(const KinematicsPluginInfo& plugin_info,
                                                           const tesseract_scene_graph::SceneGraph& scene_graph,
                                                           const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = inv_kin_factories_.find(plugin_info.class_name);
    if (it != inv_kin_factories_.end())
      return it->second->create(plugin_info.group, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_->instantiate<InvKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    inv_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(plugin_info.group, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

}  // namespace tesseract_kinematics
