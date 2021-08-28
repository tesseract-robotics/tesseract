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

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
KinematicsPluginFactory::~KinematicsPluginFactory() = default;

void KinematicsPluginFactory::addSearchPath(const std::string& path) { plugin_loader_->search_paths.insert(path); }

void KinematicsPluginFactory::addLibrary(const std::string& library_name)
{
  plugin_loader_->plugins.insert(library_name);
}

ForwardKinematics::UPtr KinematicsPluginFactory::createFwdKin(const std::string& plugin_name,
                                                              const std::string& name,
                                                              const tesseract_scene_graph::SceneGraph& scene_graph,
                                                              const tesseract_scene_graph::SceneState& scene_state,
                                                              const YAML::Node& config) const
{
  try
  {
    auto it = fwd_kin_factories_.find(plugin_name);
    if (it != fwd_kin_factories_.end())
      return it->second->create(name, scene_graph, scene_state, *this, config);

    auto plugin = plugin_loader_->instantiate<FwdKinFactory>(plugin_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_name.c_str());
      return nullptr;
    }
    fwd_kin_factories_[plugin_name] = plugin;
    return plugin->create(name, scene_graph, scene_state, *this, config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_name.c_str());
    return nullptr;
  }
}

InverseKinematics::UPtr KinematicsPluginFactory::createInvKin(const std::string& plugin_name,
                                                              const std::string& name,
                                                              const tesseract_scene_graph::SceneGraph& scene_graph,
                                                              const tesseract_scene_graph::SceneState& scene_state,
                                                              const YAML::Node& config) const
{
  try
  {
    auto it = inv_kin_factories_.find(plugin_name);
    if (it != inv_kin_factories_.end())
      return it->second->create(name, scene_graph, scene_state, *this, config);

    auto plugin = plugin_loader_->instantiate<InvKinFactory>(plugin_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_name.c_str());
      return nullptr;
    }
    inv_kin_factories_[plugin_name] = plugin;
    return plugin->create(name, scene_graph, scene_state, *this, config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_name.c_str());
    return nullptr;
  }
}

}  // namespace tesseract_kinematics
