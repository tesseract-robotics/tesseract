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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extenstions.h>
#include <tesseract_srdf/configs.h>

namespace tesseract_srdf
{
std::filesystem::path parseConfigFilePath(const tesseract_common::ResourceLocator& locator,
                                          const tinyxml2::XMLElement* xml_element,
                                          const std::array<int, 3>& /*version*/)
{
  std::string filename;
  int status = tesseract_common::QueryStringAttributeRequired(xml_element, "filename", filename);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error(std::string(xml_element->Value()) + ": Missing or failed to parse "
                                                                                  "'filename' attribute."));

  tesseract_common::Resource::Ptr resource = locator.locateResource(filename);
  if (resource == nullptr)
    std::throw_with_nested(
        std::runtime_error(std::string(xml_element->Value()) + ": Failed to locate resource '" + filename + "'."));

  std::filesystem::path file_path(resource->getFilePath());
  if (!std::filesystem::exists(file_path))
    std::throw_with_nested(std::runtime_error(std::string(xml_element->Value()) +
                                              ": config file does not exist: "
                                              "'" +
                                              file_path.string() + "'."));
  return file_path;
}

tesseract_common::CalibrationInfo parseCalibrationConfig(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const tesseract_common::ResourceLocator& locator,
                                                         const tinyxml2::XMLElement* xml_element,
                                                         const std::array<int, 3>& version)
{
  std::filesystem::path cal_config_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(cal_config_file_path.string());
    tesseract_common::processYamlIncludeDirective(config, locator);
  }
  // LCOV_EXCL_START
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("calibration_config: YAML failed to parse calibration config "
                                              "file '" +
                                              cal_config_file_path.string() + "'."));
  }
  // LCOV_EXCL_STOP

  const YAML::Node& cal_info = config[tesseract_common::CalibrationInfo::CONFIG_KEY];
  auto info = cal_info.as<tesseract_common::CalibrationInfo>();

  // Check to make sure calibration joints exist
  for (const auto& cal_joint : info.joints)
  {
    if (scene_graph.getJoint(cal_joint.first) == nullptr)
      std::throw_with_nested(std::runtime_error("calibration_config: joint '" + cal_joint.first + "' does not exist!"));
  }

  return info;
}

tesseract_common::KinematicsPluginInfo parseKinematicsPluginConfig(const tesseract_common::ResourceLocator& locator,
                                                                   const tinyxml2::XMLElement* xml_element,
                                                                   const std::array<int, 3>& version)
{
  std::filesystem::path kin_plugin_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(kin_plugin_file_path.string());
    tesseract_common::processYamlIncludeDirective(config, locator);
  }
  // LCOV_EXCL_START
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("kinematics_plugin_config: YAML failed to parse kinematics plugins "
                                              "file '" +
                                              kin_plugin_file_path.string() + "'."));
  }
  // LCOV_EXCL_STOP

  const YAML::Node& kin_plugin_info = config[tesseract_common::KinematicsPluginInfo::CONFIG_KEY];

  return kin_plugin_info.as<tesseract_common::KinematicsPluginInfo>();
}

tesseract_common::ContactManagersPluginInfo
parseContactManagersPluginConfig(const tesseract_common::ResourceLocator& locator,
                                 const tinyxml2::XMLElement* xml_element,
                                 const std::array<int, 3>& version)
{
  std::filesystem::path cm_plugin_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(cm_plugin_file_path.string());
    tesseract_common::processYamlIncludeDirective(config, locator);
  }
  // LCOV_EXCL_START
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("contact_managers_plugin_config: YAML failed to parse contact "
                                              "managers plugins "
                                              "file '" +
                                              cm_plugin_file_path.string() + "'."));
  }
  // LCOV_EXCL_STOP

  const YAML::Node& cm_plugin_info = config[tesseract_common::ContactManagersPluginInfo::CONFIG_KEY];
  return cm_plugin_info.as<tesseract_common::ContactManagersPluginInfo>();
}
}  // namespace tesseract_srdf
