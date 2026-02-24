/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Levi Armstrong
 * @date April 30, 2025
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
#ifndef TESSERACT_COLLISION_YAML_EXTENSIONS_H
#define TESSERACT_COLLISION_YAML_EXTENSIONS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/common/yaml_extensions.h>

namespace YAML
{
//=========================== ACMOverrideType Enum ===========================
template <>
struct convert<tesseract::collision::ACMOverrideType>
{
  static Node encode(const tesseract::collision::ACMOverrideType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract::collision::ACMOverrideType, std::string> m = {
      { tesseract::collision::ACMOverrideType::NONE, "NONE" },
      { tesseract::collision::ACMOverrideType::AND, "AND" },
      { tesseract::collision::ACMOverrideType::ASSIGN, "ASSIGN" },
      { tesseract::collision::ACMOverrideType::OR, "OR" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract::collision::ACMOverrideType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract::collision::ACMOverrideType> inv = {
      { "NONE", tesseract::collision::ACMOverrideType::NONE },
      { "AND", tesseract::collision::ACMOverrideType::AND },
      { "ASSIGN", tesseract::collision::ACMOverrideType::ASSIGN },
      { "OR", tesseract::collision::ACMOverrideType::OR }
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

//=========================== ContactTestType Enum ===========================
template <>
struct convert<tesseract::collision::ContactTestType>
{
  static Node encode(const tesseract::collision::ContactTestType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract::collision::ContactTestType, std::string> m = {
      { tesseract::collision::ContactTestType::FIRST, "FIRST" },
      { tesseract::collision::ContactTestType::CLOSEST, "CLOSEST" },
      { tesseract::collision::ContactTestType::ALL, "ALL" },
      { tesseract::collision::ContactTestType::LIMITED, "LIMITED" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract::collision::ContactTestType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract::collision::ContactTestType> inv = {
      { "FIRST", tesseract::collision::ContactTestType::FIRST },
      { "CLOSEST", tesseract::collision::ContactTestType::CLOSEST },
      { "ALL", tesseract::collision::ContactTestType::ALL },
      { "LIMITED", tesseract::collision::ContactTestType::LIMITED }
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

//=========================== CollisionEvaluatorType Enum ===========================
template <>
struct convert<tesseract::collision::CollisionEvaluatorType>
{
  static Node encode(const tesseract::collision::CollisionEvaluatorType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract::collision::CollisionEvaluatorType, std::string> m = {
      { tesseract::collision::CollisionEvaluatorType::NONE, "NONE" },
      { tesseract::collision::CollisionEvaluatorType::DISCRETE, "DISCRETE" },
      { tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE, "LVS_DISCRETE" },
      { tesseract::collision::CollisionEvaluatorType::CONTINUOUS, "CONTINUOUS" },
      { tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS, "LVS_CONTINUOUS" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract::collision::CollisionEvaluatorType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract::collision::CollisionEvaluatorType> inv = {
      { "NONE", tesseract::collision::CollisionEvaluatorType::NONE },
      { "DISCRETE", tesseract::collision::CollisionEvaluatorType::DISCRETE },
      { "LVS_DISCRETE", tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE },
      { "CONTINUOUS", tesseract::collision::CollisionEvaluatorType::CONTINUOUS },
      { "LVS_CONTINUOUS", tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS }
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

//=========================== CollisionCheckProgramType Enum ===========================
template <>
struct convert<tesseract::collision::CollisionCheckProgramType>
{
  static Node encode(const tesseract::collision::CollisionCheckProgramType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<tesseract::collision::CollisionCheckProgramType, std::string> m = {
      { tesseract::collision::CollisionCheckProgramType::ALL, "ALL" },
      { tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START, "ALL_EXCEPT_START" },
      { tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END, "ALL_EXCEPT_END" },
      { tesseract::collision::CollisionCheckProgramType::START_ONLY, "START_ONLY" },
      { tesseract::collision::CollisionCheckProgramType::END_ONLY, "END_ONLY" },
      { tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY, "INTERMEDIATE_ONLY" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract::collision::CollisionCheckProgramType& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, tesseract::collision::CollisionCheckProgramType> inv = {
      { "ALL", tesseract::collision::CollisionCheckProgramType::ALL },
      { "ALL_EXCEPT_START", tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START },
      { "ALL_EXCEPT_END", tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END },
      { "START_ONLY", tesseract::collision::CollisionCheckProgramType::START_ONLY },
      { "END_ONLY", tesseract::collision::CollisionCheckProgramType::END_ONLY },
      { "INTERMEDIATE_ONLY", tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY }
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

//=========================== CollisionCheckExitType Enum ===========================
template <>
struct convert<tesseract::collision::CollisionCheckExitType>
{
  static Node encode(const tesseract::collision::CollisionCheckExitType& rhs)
  {
    static const std::map<tesseract::collision::CollisionCheckExitType, std::string> m = {
      { tesseract::collision::CollisionCheckExitType::FIRST, "FIRST" },
      { tesseract::collision::CollisionCheckExitType::ONE_PER_STEP, "ONE_PER_STEP" },
      { tesseract::collision::CollisionCheckExitType::ALL, "ALL" }
    };
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, tesseract::collision::CollisionCheckExitType& rhs)
  {
    static const std::map<std::string, tesseract::collision::CollisionCheckExitType> inv = {
      { "FIRST", tesseract::collision::CollisionCheckExitType::FIRST },
      { "ONE_PER_STEP", tesseract::collision::CollisionCheckExitType::ONE_PER_STEP },
      { "ALL", tesseract::collision::CollisionCheckExitType::ALL }
    };

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
struct convert<tesseract::collision::ContactManagerConfig>
{
  static Node encode(const tesseract::collision::ContactManagerConfig& rhs)
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

  static bool decode(const Node& node, tesseract::collision::ContactManagerConfig& rhs)
  {
    if (!node.IsMap())
      return false;

    if (const YAML::Node& n = node["default_margin"])
      rhs.default_margin = n.as<double>();

    if (const YAML::Node& n = node["pair_margin_override_type"])
      rhs.pair_margin_override_type = n.as<tesseract::collision::CollisionMarginPairOverrideType>();

    if (const YAML::Node& n = node["pair_margin_data"])
      rhs.pair_margin_data = n.as<tesseract::common::PairsCollisionMarginData>();

    if (const YAML::Node& n = node["acm_override_type"])
      rhs.acm_override_type = n.as<tesseract::collision::ACMOverrideType>();

    if (const YAML::Node& n = node["acm"])
      rhs.acm = n.as<tesseract::common::AllowedCollisionMatrix>();

    if (const YAML::Node& n = node["modify_object_enabled"])
      rhs.modify_object_enabled = n.as<std::unordered_map<std::string, bool>>();

    return true;
  }
};

//=========================== ContactRequest ===========================
template <>
struct convert<tesseract::collision::ContactRequest>
{
  static Node encode(const tesseract::collision::ContactRequest& rhs)
  {
    Node node;

    node["type"] = rhs.type;
    node["calculate_penetration"] = rhs.calculate_penetration;
    node["calculate_distance"] = rhs.calculate_distance;
    node["contact_limit"] = rhs.contact_limit;
    return node;
  }

  static bool decode(const Node& node, tesseract::collision::ContactRequest& rhs)
  {
    if (const YAML::Node& n = node["type"])
      rhs.type = n.as<tesseract::collision::ContactTestType>();

    if (const YAML::Node& n = node["calculate_penetration"])
      rhs.calculate_penetration = n.as<bool>();

    if (const YAML::Node& n = node["calculate_distance"])
      rhs.calculate_distance = n.as<bool>();

    if (const YAML::Node& n = node["contact_limit"])
      rhs.contact_limit = n.as<long>();

    return true;
  }
};

//=========================== CollisionCheckConfig ===========================
template <>
struct convert<tesseract::collision::CollisionCheckConfig>
{
  static Node encode(const tesseract::collision::CollisionCheckConfig& rhs)
  {
    Node node;

    node["contact_request"] = rhs.contact_request;
    node["type"] = rhs.type;
    node["longest_valid_segment_length"] = rhs.longest_valid_segment_length;
    node["check_program_mode"] = rhs.check_program_mode;
    node["exit_condition"] = rhs.exit_condition;

    return node;
  }

  static bool decode(const Node& node, tesseract::collision::CollisionCheckConfig& rhs)
  {
    if (const YAML::Node& n = node["contact_request"])
      rhs.contact_request = n.as<tesseract::collision::ContactRequest>();
    if (const YAML::Node& n = node["type"])
      rhs.type = n.as<tesseract::collision::CollisionEvaluatorType>();
    if (const YAML::Node& n = node["longest_valid_segment_length"])
      rhs.longest_valid_segment_length = n.as<double>();
    if (const YAML::Node& n = node["check_program_mode"])
      rhs.check_program_mode = n.as<tesseract::collision::CollisionCheckProgramType>();
    if (const YAML::Node& n = node["exit_condition"])
      rhs.exit_condition = n.as<tesseract::collision::CollisionCheckExitType>();
    return true;
  }
};

}  // namespace YAML

#endif  // TESSERACT_COLLISION_YAML_EXTENSIONS_H
