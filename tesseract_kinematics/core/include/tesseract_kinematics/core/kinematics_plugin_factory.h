/**
 * @file kinematics_plugin_factory.h
 * @brief Kinematics Plugin Factory
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
#ifndef TESSERACT_KINEMATICS_KINEMATICS_PLUGIN_FACTORY_H
#define TESSERACT_KINEMATICS_KINEMATICS_PLUGIN_FACTORY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <map>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/fwd.h>
#include <tesseract_state_solver/fwd.h>
#include <tesseract_common/fwd.h>
#include <tesseract_common/plugin_info.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost_plugin_loader/macros.h>
#include <filesystem>

// clang-format off
#define TESSERACT_ADD_FWD_KIN_PLUGIN(DERIVED_CLASS, ALIAS)                                                             \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, FwdKin)

#define TESSERACT_ADD_INV_KIN_PLUGIN(DERIVED_CLASS, ALIAS)                                                             \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, InvKin)
// clang-format on

namespace tesseract_kinematics
{
/** @brief Forward declare Plugin Factory */
class KinematicsPluginFactory;
class InverseKinematics;
class ForwardKinematics;

/** @brief Define a inverse kinematics plugin which the factory can create an instance */
class InvKinFactory
{
public:
  using Ptr = std::shared_ptr<InvKinFactory>;
  using ConstPtr = std::shared_ptr<const InvKinFactory>;

  virtual ~InvKinFactory() = default;

  /**
   * @brief Create Inverse Kinematics Object
   * @param solver_name The solver name of the kinematic object
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param plugin_factory Provide access to the plugin factory so plugins and load plugins
   * @return If failed to create, nullptr is returned.
   */
  virtual std::unique_ptr<InverseKinematics> create(const std::string& solver_name,
                                                    const tesseract_scene_graph::SceneGraph& scene_graph,
                                                    const tesseract_scene_graph::SceneState& scene_state,
                                                    const KinematicsPluginFactory& plugin_factory,
                                                    const YAML::Node& config) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

/** @brief Define a forward kinematics plugin which the factory can create an instance */
class FwdKinFactory
{
public:
  using Ptr = std::shared_ptr<FwdKinFactory>;
  using ConstPtr = std::shared_ptr<const FwdKinFactory>;

  virtual ~FwdKinFactory() = default;

  /**
   * @brief Create Inverse Kinematics Object
   * @param solver_name The solver name of the kinematic object
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param plugin_factory Provide access to the plugin factory so plugins and load plugins
   * @return If failed to create, nullptr is returned.
   */
  virtual std::unique_ptr<ForwardKinematics> create(const std::string& solver_name,
                                                    const tesseract_scene_graph::SceneGraph& scene_graph,
                                                    const tesseract_scene_graph::SceneState& scene_state,
                                                    const KinematicsPluginFactory& plugin_factory,
                                                    const YAML::Node& config) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

class KinematicsPluginFactory
{
public:
  KinematicsPluginFactory();
  ~KinematicsPluginFactory();
  KinematicsPluginFactory(const KinematicsPluginFactory&) = default;
  KinematicsPluginFactory& operator=(const KinematicsPluginFactory&) = default;
  KinematicsPluginFactory(KinematicsPluginFactory&&) = default;
  KinematicsPluginFactory& operator=(KinematicsPluginFactory&&) = default;

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  KinematicsPluginFactory(YAML::Node config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  KinematicsPluginFactory(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  KinematicsPluginFactory(const std::string& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Add location for the plugin loader to search
   * @param path The full path to the directory
   */
  void addSearchPath(const std::string& path);

  /**
   * @brief Get the plugin search paths
   * @return The search paths
   */
  std::vector<std::string> getSearchPaths() const;

  /**
   * @brief Add a library to search for plugin name
   * @param library_name The library name without the prefix or suffix
   */
  void addSearchLibrary(const std::string& library_name);

  /**
   * @brief Get the plugin search libraries
   * @return The search libraries
   */
  std::vector<std::string> getSearchLibraries() const;

  /**
   * @brief Add a forward kinematics plugin to the manager
   * @param group_name The group name
   * @param solver_name The solver name
   * @param plugin_info The plugin information
   */
  void addFwdKinPlugin(const std::string& group_name,
                       const std::string& solver_name,
                       tesseract_common::PluginInfo plugin_info);

  /**
   * @brief Get the map of forward kinematic plugins
   * @return A map of plugins
   */
  std::map<std::string, tesseract_common::PluginInfoContainer> getFwdKinPlugins() const;

  /**
   * @brief remove forward kinematics plugin from the manager
   * @param group_name The group name
   * @param solver_name The solver name
   */
  void removeFwdKinPlugin(const std::string& group_name, const std::string& solver_name);

  /**
   * @brief Set a groups default forward kinematics solver
   * @details A group can have multiple inverse kinematics solvers
   * @param group_name The group name
   * @param solver_name The solver name
   */
  void setDefaultFwdKinPlugin(const std::string& group_name, const std::string& solver_name);

  /**
   * @brief Get the default forward kinematics solver for a group
   * @param group_name The group
   * @return The default solver name
   */
  std::string getDefaultFwdKinPlugin(const std::string& group_name) const;

  /**
   * @brief Add a inverse kinematics plugin to the manager
   * @param group_name The group name
   * @param solver_name The solver name
   * @param plugin_info The plugin information
   */
  void addInvKinPlugin(const std::string& group_name,
                       const std::string& solver_name,
                       tesseract_common::PluginInfo plugin_info);

  /**
   * @brief Get the map of inverse kinematic plugins
   * @return A map of plugins
   */
  std::map<std::string, tesseract_common::PluginInfoContainer> getInvKinPlugins() const;

  /**
   * @brief remove inverse kinematics plugin from the manager
   * @param group_name The group name
   * @param solver_name The solver name
   */
  void removeInvKinPlugin(const std::string& group_name, const std::string& solver_name);

  /**
   * @brief Set a groups default inverse kinematics solver
   * @details A group can have multiple inverse kinematics solvers
   * @param group_name The group name
   * @param solver_name The solver name
   */
  void setDefaultInvKinPlugin(const std::string& group_name, const std::string& solver_name);

  /**
   * @brief Get the default forward inverse solver for a group
   * @param group_name The group
   * @return The default solver name
   */
  std::string getDefaultInvKinPlugin(const std::string& group_name) const;

  /**
   * @brief Get forward kinematics object given group name and solver name
   * @details This looks for kinematics plugin info added using addFwdKinPlugin. If not found nullptr is returned.
   * @param group_name The group name
   * @param solver_name The solver
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  std::unique_ptr<ForwardKinematics> createFwdKin(const std::string& group_name,
                                                  const std::string& solver_name,
                                                  const tesseract_scene_graph::SceneGraph& scene_graph,
                                                  const tesseract_scene_graph::SceneState& scene_state) const;

  /**
   * @brief Get inverse kinematics object given group name and solver name
   * @details This looks for kinematics plugin info added using addInvKinPlugin. If not found nullptr is returned.
   * @param group_name The group name
   * @param solver_name The solver
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  std::unique_ptr<InverseKinematics> createInvKin(const std::string& group_name,
                                                  const std::string& solver_name,
                                                  const tesseract_scene_graph::SceneGraph& scene_graph,
                                                  const tesseract_scene_graph::SceneState& scene_state) const;

  /**
   * @brief Get forward kinematics object given plugin info
   * @param plugin_info The plugin information to create kinematics object
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  std::unique_ptr<ForwardKinematics> createFwdKin(const std::string& solver_name,
                                                  const tesseract_common::PluginInfo& plugin_info,
                                                  const tesseract_scene_graph::SceneGraph& scene_graph,
                                                  const tesseract_scene_graph::SceneState& scene_state) const;

  /**
   * @brief Get inverse kinematics object given plugin info
   * @param plugin_info The plugin information to create kinematics object
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  std::unique_ptr<InverseKinematics> createInvKin(const std::string& solver_name,
                                                  const tesseract_common::PluginInfo& plugin_info,
                                                  const tesseract_scene_graph::SceneGraph& scene_graph,
                                                  const tesseract_scene_graph::SceneState& scene_state) const;

  /**
   * @brief Save the plugin information to a yaml config file
   * @param file_path The file path
   */
  void saveConfig(const std::filesystem::path& file_path) const;

  /**
   * @brief Get the plugin information config as a yaml node
   * @return The plugin information config yaml node/
   */
  YAML::Node getConfig() const;

private:
  mutable std::map<std::string, FwdKinFactory::Ptr> fwd_kin_factories_;
  mutable std::map<std::string, InvKinFactory::Ptr> inv_kin_factories_;
  std::map<std::string, tesseract_common::PluginInfoContainer> fwd_plugin_info_;
  std::map<std::string, tesseract_common::PluginInfoContainer> inv_plugin_info_;
  boost_plugin_loader::PluginLoader plugin_loader_;

  void loadConfig(const YAML::Node& config);
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KINEMATICS_PLUGIN_FACTORY_H
