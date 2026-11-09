/**
 * @file visualization_loader.h
 * @brief Visualization Loader
 *
 * @author Levi Armstrong
 * @date July 29, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_VISUALIZATION_VISUALIZATION_LOADER_H
#define TESSERACT_VISUALIZATION_VISUALIZATION_LOADER_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost_plugin_loader/plugin_loader.hpp>

namespace tesseract::visualization
{
class Visualization;

/**
 * @brief This is used to dynamically load tesseract visualizer.
 */
class VisualizationLoader : public boost_plugin_loader::PluginLoader
{
public:
  /** @brief This will attempt to load the tesseract_ignition visualizer provided by snap package */
  VisualizationLoader();

  /**
   * @brief Get this loader's visualization object for a plugin
   * @details The object is created by the first call and returned by later ones, so each loader owns
   * one instance per plugin. Use another loader for an independent instance. The object keeps its
   * plugin library loaded, so it may outlive this loader.
   * @param plugin_name The plugin name, if empty it tries to load default plugin
   * @param config The plugin's configuration, used by the call that creates the object
   * @return Returns nullptr if failed
   */
  std::shared_ptr<Visualization> get(std::string plugin_name = "", const YAML::Node& config = YAML::Node()) const;

private:
  mutable std::mutex instances_mutex_;
  mutable std::map<std::string, std::shared_ptr<Visualization>> instances_;
};

}  // namespace tesseract::visualization

#endif  // TESSERACT_VISUALIZATION_VISUALIZATION_LOADER_H
