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
#include <tesseract_common/plugin_loader.h>

const std::string TESSERACT_IGNITION_LIBRARY_NAME = "tesseract_visualization_ignition_visualization_plugin";
const std::string TESSERACT_IGNITION_SYMBOL_NAME = "TesseractIgnitionVisualizationPlugin";

const std::string TESSERACT_VISUALIZATION_LIBRARY_DIRECTORY_ENV = "TESSERACT_VISUALIZATION_PLUGIN_DIRECTORY";
const std::string TESSERACT_VISUALIZATION_LIBRARY_NAME_ENV = "TESSERACT_VISUALIZATION_PLUGIN_LIBRARY_NAME";
const std::string TESSERACT_VISUALIZATION_SYMBOL_NAME_ENV = "TESSERACT_VISUALIZATION_PLUGIN_SYMBOL_NAME";

namespace tesseract_visualization
{
VisualizationLoader::VisualizationLoader()
{
  // Check for environment variable to override default library
  const char* env_library_directory = std::getenv(TESSERACT_VISUALIZATION_LIBRARY_DIRECTORY_ENV.c_str());
  const char* env_library_name = std::getenv(TESSERACT_VISUALIZATION_LIBRARY_NAME_ENV.c_str());
  const char* env_symbol_name = std::getenv(TESSERACT_VISUALIZATION_SYMBOL_NAME_ENV.c_str());
  if (env_library_directory && env_library_name && env_symbol_name)
  {
    loader_ = std::make_shared<tesseract_common::PluginLoader>(std::string(env_library_directory),
                                                               std::string(env_library_name));
    symbol_name_ = std::string(env_symbol_name);
  }
  else if (env_library_name && env_symbol_name)
  {
    loader_ = std::make_shared<tesseract_common::PluginLoader>(std::string(env_library_name));
    symbol_name_ = std::string(env_symbol_name);
  }
  else
  {
    loader_ = std::make_shared<tesseract_common::PluginLoader>(std::string(TESSERACT_VISUALIZATION_PLUGIN_PATH),
                                                               std::string(TESSERACT_IGNITION_LIBRARY_NAME));
    symbol_name_ = std::string(TESSERACT_IGNITION_SYMBOL_NAME);
  }
}

VisualizationLoader::VisualizationLoader(const std::string& library_directory,
                                         const std::string& library_name,
                                         const std::string& symbol_name)
  : loader_(std::make_shared<tesseract_common::PluginLoader>(library_directory, library_name))
  , symbol_name_(symbol_name)
{
}

VisualizationLoader::VisualizationLoader(const std::string& library_name, const std::string& symbol_name)
  : loader_(std::make_shared<tesseract_common::PluginLoader>(library_name)), symbol_name_(symbol_name)
{
}

Visualization::Ptr VisualizationLoader::get()
{
  try
  {
    return loader_->createSharedInstance<Visualization>(symbol_name_);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn(
        "Failed to load symbol '%s' from library '%s'", symbol_name_.c_str(), loader_->getLibraryName().c_str());
    return nullptr;
  }
}

}  // namespace tesseract_visualization
