/**
 * @file profile_plugin_factory.h
 * @brief A plugin factory for producing a profile
 *
 * @author Levi Armstrong
 * @date August 27, 2025
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
#ifndef TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H
#define TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost_plugin_loader/fwd.h>
#include <boost_plugin_loader/macros.h>
#include <filesystem>

// clang-format off
#define TESSERACT_ADD_PROFILE_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, Profile)
// clang-format on

namespace YAML
{
class Node;
}

namespace tesseract::common
{
class Profile;
class ProfilePluginFactory;
struct ProfilesPluginInfo;
struct PluginInfo;
class ResourceLocator;
class ProfileDictionary;

/**
 * @brief This is a base class used to store data need by the profiles
 * This may contain the environment, scene graph, etc.
 */
struct ProfileFactoryData
{
  virtual ~ProfileFactoryData() = default;
};

/** @brief Profile Factory class used for loading profile to be called by name */
class ProfileFactory
{
public:
  using Ptr = std::shared_ptr<ProfileFactory>;
  using ConstPtr = std::shared_ptr<const ProfileFactory>;

  virtual ~ProfileFactory() = default;

  virtual std::unique_ptr<Profile> create(const std::string& name,
                                          const YAML::Node& config,
                                          const std::shared_ptr<const ProfileFactoryData>& data,
                                          const ProfilePluginFactory& plugin_factory) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

class ProfilePluginFactory
{
public:
  using PluginInfoMap = std::map<std::string, tesseract::common::PluginInfo>;

  ProfilePluginFactory();
  ~ProfilePluginFactory();
  ProfilePluginFactory(const ProfilePluginFactory&) = delete;
  ProfilePluginFactory& operator=(const ProfilePluginFactory&) = delete;
  ProfilePluginFactory(ProfilePluginFactory&&) noexcept;
  ProfilePluginFactory& operator=(ProfilePluginFactory&&) noexcept;

  /**
   * @brief Load plugins from a configuration object
   * @param config The config object
   */
  ProfilePluginFactory(const tesseract::common::ProfilesPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  ProfilePluginFactory(const YAML::Node& config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  ProfilePluginFactory(const std::filesystem::path& config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  ProfilePluginFactory(const std::string& config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Loads plugins from a configuration object
   * @param config the config object
   */
  void loadConfig(const tesseract::common::ProfilesPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  void loadConfig(YAML::Node config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  void loadConfig(const std::filesystem::path& config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  void loadConfig(const std::string& config, const tesseract::common::ResourceLocator& locator);

  /**
   * @brief Set the factory data passed when constructing the plugin
   * @param data The factory data
   */
  void setFactoryData(std::shared_ptr<const ProfileFactoryData> data);

  /**
   * @brief Get the factory data
   * @return The factory data passed when constructing the plugin
   */
  std::shared_ptr<const ProfileFactoryData> getFactoryData() const;

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
   * @brief Clear the search paths
   *
   */
  void clearSearchPaths();

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
   * @brief Clean the search libraries
   *
   */
  void clearSearchLibraries();

  /**
   * @brief Add a profile plugin
   * @param ns The namespace
   * @param name The name
   * @param plugin_info The plugin information
   */
  void addPlugin(const std::string& ns, const std::string& name, tesseract::common::PluginInfo plugin_info);

  /**
   * @brief Check if it has profile plugins
   * @return True if profile PluginInfoMap is not empty, otherwise fale
   */
  bool hasPlugins() const;

  /**
   * @brief Get the map of profile plugins
   * @return A map of plugins
   */
  std::map<std::string, PluginInfoMap> getPlugins() const;

  /**
   * @brief Remove profile plugin
   * @param ns The namespace
   * @param name The name of the profile to remove
   */
  void removePlugin(const std::string& ns, const std::string& name);

  /**
   * @brief Get profile object given name
   * @details This looks for profile plugin info. If not found nullptr is returned.
   * @param ns The namespace
   * @param name The name
   */
  std::unique_ptr<Profile> create(const std::string& ns, const std::string& name) const;

  /**
   * @brief Get profile object given plugin info
   * @param name The name
   * @param plugin_info The plugin information to create profile object
   */
  std::unique_ptr<Profile> create(const std::string& name, const PluginInfo& plugin_info) const;

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

  /**
   * @brief This will load all plugins into a profile dictionary
   * @return A profile dictionary
   */
  ProfileDictionary getProfileDictionary() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;

  void loadConfig(YAML::Node config);
};
}  // namespace tesseract::common
#endif  // TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H
