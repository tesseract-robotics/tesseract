/**
 * @file yaml_utils.h
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
#ifndef TESSERACT_COMMON_YAML_UTILS_H
#define TESSERACT_COMMON_YAML_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
#include <set>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace YAML
{
template <>
struct convert<tesseract_common::PluginInfo>
{
  static Node encode(const tesseract_common::PluginInfo& rhs)
  {
    Node node;
    node["class"] = rhs.class_name;
    node["default"] = rhs.is_default;
    if (!rhs.config.IsNull())
      node["config"] = rhs.config;
    return node;
  }

  static bool decode(const Node& node, tesseract_common::PluginInfo& rhs)
  {
    // Check for required entries
    if (!node["class"])
      throw std::runtime_error("PluginInfo, missing 'class' entry!");

    rhs.class_name = node["class"].as<std::string>();

    if (node["default"])
      rhs.is_default = node["default"].as<bool>();

    if (node["config"])
      rhs.config = node["config"];

    return true;
  }
};

template <typename T, typename A>
struct convert<std::set<T, A>>
{
  static Node encode(const std::set<T, A>& rhs)
  {
    Node node(NodeType::Sequence);
    for (const auto& element : rhs)
      node.push_back(element);
    return node;
  }

  static bool decode(const Node& node, std::set<T, A>& rhs)
  {
    if (!node.IsSequence())
      return false;

    rhs.clear();
    for (const auto& element : node)
      rhs.insert(element.as<std::string>());

    return true;
  }
};

template <>
struct convert<Eigen::Isometry3d>
{
  static Node encode(const Eigen::Isometry3d& rhs)
  {
    Node xyz;
    xyz["x"] = rhs.translation().x();
    xyz["y"] = rhs.translation().y();
    xyz["z"] = rhs.translation().z();

    const Eigen::Quaterniond q(rhs.linear());
    Node quat;
    quat["x"] = q.x();
    quat["y"] = q.y();
    quat["z"] = q.z();
    quat["w"] = q.w();

    Node node;
    node["position"] = xyz;
    node["orientation"] = quat;

    return node;
  }

  static bool decode(const Node& node, Eigen::Isometry3d& rhs)
  {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();

    const YAML::Node& p = node["position"];
    out.translation().x() = p["x"].as<double>();
    out.translation().y() = p["y"].as<double>();
    out.translation().z() = p["z"].as<double>();

    const YAML::Node& o = node["orientation"];
    if (o["x"] && o["y"] && o["z"] && o["w"])
    {
      Eigen::Quaterniond quat;
      quat.x() = o["x"].as<double>();
      quat.y() = o["y"].as<double>();
      quat.z() = o["z"].as<double>();
      quat.w() = o["w"].as<double>();

      out.linear() = quat.toRotationMatrix();
    }
    else if (o["r"] && o["p"] && o["y"])
    {
      double r = o["r"].as<double>();
      double p = o["p"].as<double>();
      double y = o["y"].as<double>();

      Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond rpy = yawAngle * pitchAngle * rollAngle;

      out.linear() = rpy.toRotationMatrix();
    }
    else
    {
      throw std::runtime_error("Eigen::Isometry3d, failed to decode orientation missing (x, y, z, w) or (r, p, y)");
    }

    rhs = out;
    return true;
  }
};

template <>
struct convert<Eigen::VectorXd>
{
  static Node encode(const Eigen::VectorXd& rhs)
  {
    Node node;
    for (long i = 0; i < rhs.size(); ++i)
      node.push_back(rhs(i));

    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXd& rhs)
  {
    if (!node.IsSequence())
      return false;

    rhs.resize(node.size());
    for (long i = 0; i < node.size(); ++i)
      rhs(i) = node[i].as<double>();

    return true;
  }
};

template <>
struct convert<Eigen::Vector2d>
{
  static Node encode(const Eigen::Vector2d& rhs)
  {
    Node node;
    for (long i = 0; i < rhs.size(); ++i)
      node.push_back(rhs(i));

    return node;
  }

  static bool decode(const Node& node, Eigen::Vector2d& rhs)
  {
    if (!node.IsSequence() || (node.size() != 2))
      return false;

    for (long i = 0; i < 2; ++i)
      rhs(i) = node[i].as<double>();

    return true;
  }
};
}  // namespace YAML

#endif  // TESSERACT_COMMON_YAML_UTILS_H
