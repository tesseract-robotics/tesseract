/**
 * @file collision_margins.h
 * @brief Parse config files
 *
 * @author Levi Armstrong
 * @date January 25, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Southwest Research Institute
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
#ifndef TESSERACT_SRDF_CONFIGS_H
#define TESSERACT_SRDF_CONFIGS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
}
namespace tesseract_scene_graph
{
class SceneGraph;
}

namespace tesseract_srdf
{
/**
 * @brief Parse a config xml element
 * @details It expects the element to have the attribute 'filename'
 * @param locator The locator used to process the file name URL
 * @param xml_element The xml element to process
 * @param version The SRDF version
 * @return The extracted file path
 */
tesseract_common::fs::path parseConfigFilePath(const tesseract_common::ResourceLocator& locator,
                                               const tinyxml2::XMLElement* xml_element,
                                               const std::array<int, 3>& version);

/**
 * @brief Parse calibration config xml element
 * @param scene_graph The scene graph
 * @param locator The locator used to process the file name URL
 * @param xml_element The xml element to process
 * @param version The SRDF version
 * @return The calibration information
 */
tesseract_common::CalibrationInfo parseCalibrationConfig(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const tesseract_common::ResourceLocator& locator,
                                                         const tinyxml2::XMLElement* xml_element,
                                                         const std::array<int, 3>& version);

/**
 * @brief Parse kinematics plugin config xml element
 * @param locator The locator used to process the file name URL
 * @param xml_element The xml element to process
 * @param version The SRDF version
 * @return The kinematics plugin information
 */
tesseract_common::KinematicsPluginInfo parseKinematicsPluginConfig(const tesseract_common::ResourceLocator& locator,
                                                                   const tinyxml2::XMLElement* xml_element,
                                                                   const std::array<int, 3>& version);

/**
 * @brief Parse contact managers plugin config xml element
 * @param locator The locator used to process the file name URL
 * @param xml_element The xml element to process
 * @param version The SRDF version
 * @return The contact managers plugin information
 */
tesseract_common::ContactManagersPluginInfo
parseContactManagersPluginConfig(const tesseract_common::ResourceLocator& locator,
                                 const tinyxml2::XMLElement* xml_element,
                                 const std::array<int, 3>& version);
}  // namespace tesseract_srdf
#endif  // TESSERACT_SRDF_CONFIGS_H
