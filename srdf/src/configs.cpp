/**
 * @file collision_margins.h
 * @brief Parse config files
 *
 * @author Levi Armstrong
 * @date January 25, 2022
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/srdf/configs.h>

namespace tesseract::srdf
{
std::filesystem::path parseConfigFilePath(const tesseract::common::ResourceLocator& locator,
                                          const tinyxml2::XMLElement* xml_element,
                                          const std::array<int, 3>& /*version*/)
{
  std::string filename;
  int status = tesseract::common::QueryStringAttributeRequired(xml_element, "filename", filename);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error(std::string(xml_element->Value()) + ": Missing or failed to parse "
                                                                                  "'filename' attribute."));

  tesseract::common::Resource::Ptr resource = locator.locateResource(filename);
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

tesseract::common::CalibrationInfo parseCalibrationConfig(const tesseract::scene_graph::SceneGraph& scene_graph,
                                                          const tesseract::common::ResourceLocator& locator,
                                                          const tinyxml2::XMLElement* xml_element,
                                                          const std::array<int, 3>& version)
{
  std::filesystem::path cal_config_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(cal_config_file_path.string());
    tesseract::common::processYamlIncludeDirective(config, locator);
  }
  // LCOV_EXCL_START
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("calibration_config: YAML failed to parse calibration config "
                                              "file '" +
                                              cal_config_file_path.string() + "'."));
  }
  // LCOV_EXCL_STOP

  const YAML::Node& cal_info = config[tesseract::common::CalibrationInfo::CONFIG_KEY];
  auto info = cal_info.as<tesseract::common::CalibrationInfo>();

  // Check to make sure calibration joints exist
  for (const auto& cal_joint : info.joints)
  {
    if (scene_graph.getJoint(cal_joint.first) == nullptr)
      std::throw_with_nested(std::runtime_error("calibration_config: joint '" + cal_joint.first + "' does not exist!"));
  }

  return info;
}

tesseract::common::KinematicsPluginInfo parseKinematicsPluginConfig(const tesseract::common::ResourceLocator& locator,
                                                                    const tinyxml2::XMLElement* xml_element,
                                                                    const std::array<int, 3>& version)
{
  std::filesystem::path kin_plugin_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(kin_plugin_file_path.string());
    tesseract::common::processYamlIncludeDirective(config, locator);
  }
  // LCOV_EXCL_START
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("kinematics_plugin_config: YAML failed to parse kinematics plugins "
                                              "file '" +
                                              kin_plugin_file_path.string() + "'."));
  }
  // LCOV_EXCL_STOP

  const YAML::Node& kin_plugin_info = config[tesseract::common::KinematicsPluginInfo::CONFIG_KEY];

  return kin_plugin_info.as<tesseract::common::KinematicsPluginInfo>();
}

tesseract::common::ContactManagersPluginInfo
parseContactManagersPluginConfig(const tesseract::common::ResourceLocator& locator,
                                 const tinyxml2::XMLElement* xml_element,
                                 const std::array<int, 3>& version)
{
  std::filesystem::path cm_plugin_file_path = parseConfigFilePath(locator, xml_element, version);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(cm_plugin_file_path.string());
    tesseract::common::processYamlIncludeDirective(config, locator);
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

  const YAML::Node& cm_plugin_info = config[tesseract::common::ContactManagersPluginInfo::CONFIG_KEY];
  return cm_plugin_info.as<tesseract::common::ContactManagersPluginInfo>();
}
}  // namespace tesseract::srdf
