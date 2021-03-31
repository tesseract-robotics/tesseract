/**
 * @file visualization_loader.h
 * @brief Visualization Loader
 *
 * @author Levi Armstrong
 * @date July 29, 2020
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>
#include <tesseract_common/plugin_loader.h>

namespace tesseract_visualization
{
/**
 * @brief This is used to dynamically load tesseract visualizer.
 * This class must remain around for the life of the loaded visualization class.
 */
class VisualizationLoader : public tesseract_common::PluginLoader
{
public:
  /** @brief This will attempt to load the tesseract_ignition visualizer provided by snap package */
  VisualizationLoader();

  /**
   * @brief Load the default visualization plugin
   * @param plugin_name The plugin name, if empty it tries to load default plugin
   * @return Returns nullptr if failed
   */
  Visualization::Ptr get(std::string plugin_name = "") const;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_VISUALIZATION_LOADER_H
