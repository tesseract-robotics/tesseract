/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Levi Armstrong
 * @date April 30, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_COLLISION_CORE_YAML_EXTENSIONS_H
#define TESSERACT_COLLISION_CORE_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_common/yaml_extensions.h>

namespace YAML
{
//=========================== ACMOverrideType Enum ===========================
template <>
struct convert<tesseract_collision::ACMOverrideType>
{
  static Node encode(const tesseract_collision::ACMOverrideType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract_collision::ACMOverrideType, std::string> m = {
      { tesseract_collision::ACMOverrideType::NONE, "NONE" },
      { tesseract_collision::ACMOverrideType::AND, "AND" },
      { tesseract_collision::ACMOverrideType::ASSIGN, "ASSIGN" },
      { tesseract_collision::ACMOverrideType::OR, "OR" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract_collision::ACMOverrideType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract_collision::ACMOverrideType> inv = {
      { "NONE", tesseract_collision::ACMOverrideType::NONE },
      { "AND", tesseract_collision::ACMOverrideType::AND },
      { "ASSIGN", tesseract_collision::ACMOverrideType::ASSIGN },
      { "OR", tesseract_collision::ACMOverrideType::OR }
    };
    // LCOV_EXCL_STOP

    if (!node.IsScalar())
      return false;

    auto it = inv.find(node.Scalar());
    if (it == inv.end())
      return false;

    rhs = it->second;
    return true;
  }
};

//=========================== ContactManagerConfig ===========================
template <>
struct convert<tesseract_collision::ContactManagerConfig>
{
  static Node encode(const tesseract_collision::ContactManagerConfig& rhs)
  {
    Node node;
    if (rhs.default_margin.has_value())
      node["default_margin"] = rhs.default_margin.value();

    node["pair_margin_override_type"] = rhs.pair_margin_override_type;
    node["pair_margin_data"] = rhs.pair_margin_data;
    node["acm_override_type"] = rhs.acm_override_type;
    node["acm"] = rhs.acm;
    node["modify_object_enabled"] = rhs.modify_object_enabled;
    return node;
  }

  static bool decode(const Node& node, tesseract_collision::ContactManagerConfig& rhs)
  {
    if (!node.IsMap())
      return false;

    if (const YAML::Node& n = node["default_margin"])
      rhs.default_margin = n.as<double>();

    if (const YAML::Node& n = node["pair_margin_override_type"])
      rhs.pair_margin_override_type = n.as<tesseract_collision::CollisionMarginPairOverrideType>();

    if (const YAML::Node& n = node["pair_margin_data"])
      rhs.pair_margin_data = n.as<tesseract_common::PairsCollisionMarginData>();

    if (const YAML::Node& n = node["acm_override_type"])
      rhs.acm_override_type = n.as<tesseract_collision::ACMOverrideType>();

    if (const YAML::Node& n = node["acm"])
      rhs.acm = n.as<tesseract_common::AllowedCollisionMatrix>();

    if (const YAML::Node& n = node["modify_object_enabled"])
      rhs.modify_object_enabled = n.as<std::unordered_map<std::string, bool>>();

    return true;
  }
};

}  // namespace YAML

#endif  // TESSERACT_COLLISION_CORE_YAML_EXTENSIONS_H
