/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Levi Armstrong
 * @date September 5, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_common/schema_registration.h>

TESSERACT_REGISTER_SCHEMA(Eigen::Isometry3d, YAML::convert<Eigen::Isometry3d>::schema)
TESSERACT_REGISTER_SCHEMA(Eigen::VectorXd, YAML::convert<Eigen::VectorXd>::schema)
TESSERACT_REGISTER_SCHEMA(Eigen::Vector2d, YAML::convert<Eigen::Vector2d>::schema)
TESSERACT_REGISTER_SCHEMA(Eigen::Vector3d, YAML::convert<Eigen::Vector3d>::schema)
// TESSERACT_REGISTER_SCHEMA(tesseract_common::PluginInfo, YAML::convert<tesseract_common::PluginInfo>::schema)
// TESSERACT_REGISTER_SCHEMA(tesseract_common::PluginInfoContainer,
// YAML::convert<tesseract_common::PluginInfoContainer>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::KinematicsPluginInfo,
                          YAML::convert<tesseract_common::KinematicsPluginInfo>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::ContactManagersPluginInfo,
                          YAML::convert<tesseract_common::ContactManagersPluginInfo>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::TaskComposerPluginInfo,
                          YAML::convert<tesseract_common::TaskComposerPluginInfo>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::CalibrationInfo, YAML::convert<tesseract_common::CalibrationInfo>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::TransformMap, YAML::convert<tesseract_common::TransformMap>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::Toolpath, YAML::convert<tesseract_common::Toolpath>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::CollisionMarginPairOverrideType,
                          YAML::convert<tesseract_common::CollisionMarginPairOverrideType>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::PairsCollisionMarginData,
                          YAML::convert<tesseract_common::PairsCollisionMarginData>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::CollisionMarginPairData,
                          YAML::convert<tesseract_common::CollisionMarginPairData>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::AllowedCollisionEntries,
                          YAML::convert<tesseract_common::AllowedCollisionEntries>::schema)
TESSERACT_REGISTER_SCHEMA(tesseract_common::AllowedCollisionMatrix,
                          YAML::convert<tesseract_common::AllowedCollisionMatrix>::schema)
