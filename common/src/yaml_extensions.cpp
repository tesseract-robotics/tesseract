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

#include <tesseract/common/yaml_extensions.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>

using namespace tesseract::common;
using namespace tesseract::common::property_attribute;
using namespace tesseract::common::property_type;

// ================================ Eigen::Isometry3d ================================
PropertyTree YAML::convert<Eigen::Isometry3d>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .container("position").required()
          .doubleNum("x").required().done()
          .doubleNum("y").required().done()
          .doubleNum("z").required().done()
      .done()
      .container("orientation").required()
          .doubleNum("x").done()  // quaternion x
          .doubleNum("y").done()  // quaternion y  (shared with rpy yaw)
          .doubleNum("z").done()  // quaternion z
          .doubleNum("w").done()  // quaternion w
          .doubleNum("r").done()  // rpy roll
          .doubleNum("p").done()  // rpy pitch
      .done()
      .build();
  // clang-format on
}

// ================================ Eigen::VectorXd ================================
PropertyTree YAML::convert<Eigen::VectorXd>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, EIGEN_VECTOR_XD).build();
}

// ================================ Eigen::Vector2d ================================
PropertyTree YAML::convert<Eigen::Vector2d>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, EIGEN_VECTOR_2D).build();
}

// ================================ Eigen::Vector3d ================================
PropertyTree YAML::convert<Eigen::Vector3d>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, EIGEN_VECTOR_3D).build();
}

// ================================ KinematicsPluginInfo ================================
PropertyTree YAML::convert<tesseract::common::KinematicsPluginInfo>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .customType("search_paths", createList(STRING)).done()
      .customType("search_libraries", createList(STRING)).done()
      .customType("fwd_kin_plugins", createMap("tesseract::common::PluginInfoContainer"))
          .validator(validateCustomType).done()
      .customType("inv_kin_plugins", createMap("tesseract::common::PluginInfoContainer"))
          .validator(validateCustomType).done()
      .build();
  // clang-format on
}

// ================================ ContactManagersPluginInfo ================================
PropertyTree YAML::convert<tesseract::common::ContactManagersPluginInfo>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .customType("search_paths", createList(STRING)).done()
      .customType("search_libraries", createList(STRING)).done()
      .customType("discrete_plugins", "tesseract::common::PluginInfoContainer")
          .validator(validateCustomType).done()
      .customType("continuous_plugins", "tesseract::common::PluginInfoContainer")
          .validator(validateCustomType).done()
      .build();
  // clang-format on
}

// ================================ TaskComposerPluginInfo ================================
PropertyTree YAML::convert<tesseract::common::TaskComposerPluginInfo>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .customType("search_paths", createList(STRING)).done()
      .customType("search_libraries", createList(STRING)).done()
      .customType("executors", createMap("tesseract::common::PluginInfoContainer"))
          .validator(validateCustomType).done()
      .customType("tasks", createMap("tesseract::common::PluginInfoContainer"))
          .validator(validateCustomType).done()
      .build();
  // clang-format on
}

// ================================ JointIdTransformMap ================================
PropertyTree YAML::convert<tesseract::common::JointIdTransformMap>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, createMap(EIGEN_ISOMETRY_3D)).build();
}

// ================================ CalibrationInfo ================================
PropertyTree YAML::convert<tesseract::common::CalibrationInfo>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .customType("joints", "tesseract::common::JointIdTransformMap").required().done()
      .build();
  // clang-format on
}

// ================================ Toolpath ================================
PropertyTree YAML::convert<tesseract::common::Toolpath>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, createList(EIGEN_ISOMETRY_3D)).build();
}

// ================================ CollisionMarginPairOverrideType ================================
PropertyTree YAML::convert<tesseract::common::CollisionMarginPairOverrideType>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, STRING)
      .enumValues({ "NONE", "MODIFY", "REPLACE" })
      .build();
  // clang-format on
}

// ================================ PairsCollisionMarginData ================================
PropertyTree YAML::convert<tesseract::common::PairsCollisionMarginData>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, "tesseract::common::PairsCollisionMarginData")
      .validator([](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
        const YAML::Node& yn = node.getValue();
        if (!yn.IsMap())
        {
          errors.push_back(path + ": PairsCollisionMarginData must be a map");
          return;
        }
        for (auto it = yn.begin(); it != yn.end(); ++it)
        {
          YAML::Node key_node = it->first;
          if (!key_node.IsSequence() || key_node.size() != 2)
            errors.push_back(path + ": PairsCollisionMarginData key must be a sequence of size 2");
          try { it->second.as<double>(); }
          catch (const std::exception& e) { errors.push_back(path + ": " + e.what()); }
        }
      })
      .build();
  // clang-format on
}

// ================================ CollisionMarginPairData ================================
PropertyTree YAML::convert<tesseract::common::CollisionMarginPairData>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, "tesseract::common::PairsCollisionMarginData").build();
}

// ================================ AllowedCollisionEntries ================================
PropertyTree YAML::convert<tesseract::common::AllowedCollisionEntries>::schema()
{
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(TYPE, "tesseract::common::AllowedCollisionEntries")
      .validator([](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
        const YAML::Node& yn = node.getValue();
        if (!yn.IsMap())
        {
          errors.push_back(path + ": AllowedCollisionEntries must be a map");
          return;
        }
        for (auto it = yn.begin(); it != yn.end(); ++it)
        {
          YAML::Node key_node = it->first;
          if (!key_node.IsSequence() || key_node.size() != 2)
            errors.push_back(path + ": AllowedCollisionEntries key must be a sequence of size 2");
          try { it->second.as<std::string>(); }
          catch (const std::exception& e) { errors.push_back(path + ": " + e.what()); }
        }
      })
      .build();
  // clang-format on
}

// ================================ AllowedCollisionMatrix ================================
PropertyTree YAML::convert<tesseract::common::AllowedCollisionMatrix>::schema()
{
  return PropertyTreeBuilder().attribute(TYPE, "tesseract::common::AllowedCollisionEntries").build();
}

TESSERACT_SCHEMA_REGISTER(Eigen::Isometry3d, YAML::convert<Eigen::Isometry3d>::schema)
TESSERACT_SCHEMA_REGISTER(Eigen::VectorXd, YAML::convert<Eigen::VectorXd>::schema)
TESSERACT_SCHEMA_REGISTER(Eigen::Vector2d, YAML::convert<Eigen::Vector2d>::schema)
TESSERACT_SCHEMA_REGISTER(Eigen::Vector3d, YAML::convert<Eigen::Vector3d>::schema)
// TESSERACT_REGISTER_SCHEMA(tesseract::common::PluginInfo, YAML::convert<tesseract::common::PluginInfo>::schema)
// TESSERACT_REGISTER_SCHEMA(tesseract::common::PluginInfoContainer,
// YAML::convert<tesseract::common::PluginInfoContainer>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::KinematicsPluginInfo,
                          YAML::convert<tesseract::common::KinematicsPluginInfo>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::ContactManagersPluginInfo,
                          YAML::convert<tesseract::common::ContactManagersPluginInfo>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::TaskComposerPluginInfo,
                          YAML::convert<tesseract::common::TaskComposerPluginInfo>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::CalibrationInfo, YAML::convert<tesseract::common::CalibrationInfo>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::JointIdTransformMap,
                          YAML::convert<tesseract::common::JointIdTransformMap>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::Toolpath, YAML::convert<tesseract::common::Toolpath>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::CollisionMarginPairOverrideType,
                          YAML::convert<tesseract::common::CollisionMarginPairOverrideType>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::PairsCollisionMarginData,
                          YAML::convert<tesseract::common::PairsCollisionMarginData>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::CollisionMarginPairData,
                          YAML::convert<tesseract::common::CollisionMarginPairData>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::AllowedCollisionEntries,
                          YAML::convert<tesseract::common::AllowedCollisionEntries>::schema)
TESSERACT_SCHEMA_REGISTER(tesseract::common::AllowedCollisionMatrix,
                          YAML::convert<tesseract::common::AllowedCollisionMatrix>::schema)
