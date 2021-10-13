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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_common/class_loader.h>

const std::string TESSERACT_IGNITION_LIBRARY_NAME = "tesseract_visualization_ignition_visualization_plugin";
const std::string TESSERACT_IGNITION_SYMBOL_NAME = "TesseractIgnitionVisualizationPlugin";

const std::string TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES_ENV = "TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES";
const std::string TESSERACT_VISUALIZATION_PLUGINS_ENV = "TESSERACT_VISUALIZATION_PLUGINS";

namespace tesseract_visualization
{
VisualizationLoader::VisualizationLoader()
{
  search_paths_env = TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES_ENV;
  search_libraries_env = TESSERACT_VISUALIZATION_PLUGINS_ENV;
  search_libraries.insert(TESSERACT_IGNITION_LIBRARY_NAME);
  search_paths.insert(TESSERACT_VISUALIZATION_PLUGIN_PATH);
}

Visualization::Ptr VisualizationLoader::get(std::string plugin_name) const
{
  if (plugin_name.empty())
    plugin_name = TESSERACT_IGNITION_SYMBOL_NAME;

  try
  {
    return instantiate<Visualization>(plugin_name);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_name.c_str());
    return nullptr;
  }
}

}  // namespace tesseract_visualization
