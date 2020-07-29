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

const std::string TESSERACT_IGNITION_LIBRARY = "/snap/tesseract-ignition/current/opt/ros/melodic/lib/"
                                               "libTesseractIgnitionVisualization.so";
const std::string TESSERACT_IGNITION_CLASS = "tesseract_ignition::TesseractIgnitionVisualization";

namespace tesseract_visualization
{
VisualizationLoader::VisualizationLoader()
  : library_path_(TESSERACT_IGNITION_LIBRARY), derived_class_(TESSERACT_IGNITION_CLASS)
{
  createLoader(TESSERACT_IGNITION_LIBRARY);
}

VisualizationLoader::VisualizationLoader(const std::string& library_path, const std::string& derived_class)
  : library_path_(library_path), derived_class_(derived_class)
{
  createLoader(library_path);
}

Visualization::Ptr VisualizationLoader::get()
{
  // Dynamically load visualization library
  if (loader_)
  {
    try
    {
      if (loader_->isClassAvailable<tesseract_visualization::Visualization>(derived_class_))
        return loader_->createSharedInstance<tesseract_visualization::Visualization>(derived_class_);
    }
    catch (const std::exception&)
    {
      CONSOLE_BRIDGE_logWarn(
          "Failed to load class '%s' from library '%s'", library_path_.c_str(), derived_class_.c_str());
      return nullptr;
    }
  }

  return nullptr;
}

void VisualizationLoader::createLoader(const std::string& library_path)
{
  try
  {
    loader_ = std::make_shared<class_loader::ClassLoader>(library_path);
  }
  catch (const std::exception&)
  {
    loader_ = nullptr;
    CONSOLE_BRIDGE_logWarn("Failed to load library '%s'", library_path_.c_str());
  }
}

}  // namespace tesseract_visualization
