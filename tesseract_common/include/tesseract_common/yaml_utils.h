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
#include <array>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_common
{
/**
 * @brief Converts a YAML::Node to a yaml string
 * @param node Input node
 * @return String containing the yaml
 */
inline std::string toYAMLString(const YAML::Node& node)
{
  std::stringstream stream;
  stream << node;
  return stream.str();
}
/**
 * @brief Converts yaml string to a YAML::Node
 * @param string Input string containing the yaml
 * @return Resulting YAML::Node
 */
inline YAML::Node fromYAMLString(const std::string& string) { return YAML::Load(string); }
/**
 * @brief Checks if the YAML::Nodes are identical
 * @details The == operator checks if they reference the same memory. This checks if they contain the same information
 * @param node1 Input YAML::Node
 * @param node2 Input YAML::Node
 */
inline bool isIdentical(const YAML::Node& node1, const YAML::Node& node2)
{
  return toYAMLString(node1) == toYAMLString(node2);
}
}  // namespace tesseract_common

namespace YAML
{
template <>
struct convert<tesseract_common::PluginInfo>
{
  static Node encode(const tesseract_common::PluginInfo& rhs)
  {
    Node node;
    node["class"] = rhs.class_name;

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

    if (node["config"])  // NOLINT
      rhs.config = node["config"];

    return true;
  }
};

template <>
struct convert<tesseract_common::PluginInfoContainer>
{
  static Node encode(const tesseract_common::PluginInfoContainer& rhs)
  {
    Node node;
    if (!rhs.default_plugin.empty())
      node["default"] = rhs.default_plugin;

    node["plugins"] = rhs.plugins;

    return node;
  }

  static bool decode(const Node& node, tesseract_common::PluginInfoContainer& rhs)
  {
    if (node["default"])  // NOLINT
      rhs.default_plugin = node["default"].as<std::string>();

    if (!node["plugins"])  // NOLINT
      throw std::runtime_error("PluginInfoContainer, missing 'plugins' entry!");

    const Node& plugins = node["plugins"];
    if (!plugins.IsMap())
      throw std::runtime_error("PluginInfoContainer, 'plugins' should contain a map of plugins!");

    try
    {
      rhs.plugins = plugins.as<tesseract_common::PluginInfoMap>();
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(std::string("PluginInfoContainer: Constructor failed to cast 'plugins' to "
                                           "tesseract_common::PluginInfoMap! Details: ") +
                               e.what());
    }

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
    if (o["x"] && o["y"] && o["z"] && o["w"])  // NOLINT
    {
      Eigen::Quaterniond quat;
      quat.x() = o["x"].as<double>();
      quat.y() = o["y"].as<double>();
      quat.z() = o["z"].as<double>();
      quat.w() = o["w"].as<double>();
      quat.normalize();

      out.linear() = quat.toRotationMatrix();
    }
    else if (o["r"] && o["p"] && o["y"])  // NOLINT
    {
      auto r = o["r"].as<double>();
      auto p = o["p"].as<double>();
      auto y = o["y"].as<double>();

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

    rhs.resize(static_cast<Eigen::Index>(node.size()));
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

template <>
struct convert<tesseract_common::KinematicsPluginInfo>
{
  static Node encode(const tesseract_common::KinematicsPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string FWD_KIN_PLUGINS_KEY{ "fwd_kin_plugins" };
    const std::string INV_KIN_PLUGINS_KEY{ "inv_kin_plugins" };

    YAML::Node kinematic_plugins;
    if (!rhs.search_paths.empty())
      kinematic_plugins[SEARCH_PATHS_KEY] = rhs.search_paths;

    if (!rhs.search_libraries.empty())
      kinematic_plugins[SEARCH_LIBRARIES_KEY] = rhs.search_libraries;

    if (!rhs.fwd_plugin_infos.empty())
      kinematic_plugins[FWD_KIN_PLUGINS_KEY] = rhs.fwd_plugin_infos;

    if (!rhs.inv_plugin_infos.empty())
      kinematic_plugins[INV_KIN_PLUGINS_KEY] = rhs.inv_plugin_infos;

    return kinematic_plugins;
  }

  static bool decode(const Node& node, tesseract_common::KinematicsPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string FWD_KIN_PLUGINS_KEY{ "fwd_kin_plugins" };
    const std::string INV_KIN_PLUGINS_KEY{ "inv_kin_plugins" };

    if (const YAML::Node& search_paths = node[SEARCH_PATHS_KEY])
    {
      std::set<std::string> sp;
      try
      {
        sp = search_paths.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("KinematicsPluginFactory: Constructor failed to cast '" + SEARCH_PATHS_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_paths.insert(sp.begin(), sp.end());
    }

    if (const YAML::Node& search_libraries = node[SEARCH_LIBRARIES_KEY])
    {
      std::set<std::string> sl;
      try
      {
        sl = search_libraries.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("KinematicsPluginFactory: Constructor failed to cast '" + SEARCH_LIBRARIES_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_libraries.insert(sl.begin(), sl.end());
    }

    if (const YAML::Node& fwd_kin_plugins = node[FWD_KIN_PLUGINS_KEY])
    {
      if (!fwd_kin_plugins.IsMap())
        throw std::runtime_error(FWD_KIN_PLUGINS_KEY + ", should contain a map of group names to solver plugins!");

      try
      {
        rhs.fwd_plugin_infos = fwd_kin_plugins.as<std::map<std::string, tesseract_common::PluginInfoContainer>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("KinematicsPluginFactory: Constructor failed to cast '" + FWD_KIN_PLUGINS_KEY +
                                 "' to std::map<std::string, "
                                 "tesseract_common::PluginInfoContainer>! Details: " +
                                 e.what());
      }
    }

    if (const YAML::Node& inv_kin_plugins = node[INV_KIN_PLUGINS_KEY])
    {
      if (!inv_kin_plugins.IsMap())
        throw std::runtime_error(INV_KIN_PLUGINS_KEY + ", should contain a map of group names to solver plugins!");

      try
      {
        rhs.inv_plugin_infos = inv_kin_plugins.as<std::map<std::string, tesseract_common::PluginInfoContainer>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("KinematicsPluginFactory: Constructor failed to cast '" + INV_KIN_PLUGINS_KEY +
                                 "' to std::map<std::string, "
                                 "tesseract_common::PluginInfoContainer>! Details: " +
                                 e.what());
      }
    }

    return true;
  }
};

template <>
struct convert<tesseract_common::ContactManagersPluginInfo>
{
  static Node encode(const tesseract_common::ContactManagersPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string DISCRETE_PLUGINS_KEY{ "discrete_plugins" };
    const std::string CONTINUOUS_PLUGINS_KEY{ "continuous_plugins" };

    YAML::Node contact_manager_plugins;
    if (!rhs.search_paths.empty())
      contact_manager_plugins[SEARCH_PATHS_KEY] = rhs.search_paths;

    if (!rhs.search_libraries.empty())
      contact_manager_plugins[SEARCH_LIBRARIES_KEY] = rhs.search_libraries;

    if (!rhs.discrete_plugin_infos.plugins.empty())
      contact_manager_plugins[DISCRETE_PLUGINS_KEY] = rhs.discrete_plugin_infos;

    if (!rhs.continuous_plugin_infos.plugins.empty())
      contact_manager_plugins[CONTINUOUS_PLUGINS_KEY] = rhs.continuous_plugin_infos;

    return contact_manager_plugins;
  }

  static bool decode(const Node& node, tesseract_common::ContactManagersPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string DISCRETE_PLUGINS_KEY{ "discrete_plugins" };
    const std::string CONTINUOUS_PLUGINS_KEY{ "continuous_plugins" };

    if (const YAML::Node& search_paths = node[SEARCH_PATHS_KEY])
    {
      std::set<std::string> sp;
      try
      {
        sp = search_paths.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("ContactManagersPluginFactory: Constructor failed to cast '" + SEARCH_PATHS_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_paths.insert(sp.begin(), sp.end());
    }

    if (const YAML::Node& search_libraries = node[SEARCH_LIBRARIES_KEY])
    {
      std::set<std::string> sl;
      try
      {
        sl = search_libraries.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("ContactManagersPluginFactory: Constructor failed to cast '" + SEARCH_LIBRARIES_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_libraries.insert(sl.begin(), sl.end());
    }

    if (const YAML::Node& discrete_plugins = node[DISCRETE_PLUGINS_KEY])
    {
      if (!discrete_plugins.IsMap())
        throw std::runtime_error(DISCRETE_PLUGINS_KEY + ", should contain a map of contact manager names to plugins!");

      try
      {
        rhs.discrete_plugin_infos = discrete_plugins.as<tesseract_common::PluginInfoContainer>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("ContactManagersPluginFactory: Constructor failed to cast '" + DISCRETE_PLUGINS_KEY +
                                 "' to tesseract_common::PluginInfoContainer! Details: " + e.what());
      }
    }

    if (const YAML::Node& continuous_plugins = node[CONTINUOUS_PLUGINS_KEY])
    {
      if (!continuous_plugins.IsMap())
        throw std::runtime_error(CONTINUOUS_PLUGINS_KEY + ", should contain a map of names to plugins!");

      try
      {
        rhs.continuous_plugin_infos = continuous_plugins.as<tesseract_common::PluginInfoContainer>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("ContactManagersPluginFactory: Constructor failed to cast '" + CONTINUOUS_PLUGINS_KEY +
                                 "' to tesseract_common::PluginInfoContainer! Details: " + e.what());
      }
    }

    return true;
  }
};

template <>
struct convert<tesseract_common::TaskComposerPluginInfo>
{
  static Node encode(const tesseract_common::TaskComposerPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string EXECUTOR_PLUGINS_KEY{ "executor_plugins" };
    const std::string NODE_PLUGINS_KEY{ "node_plugins" };

    YAML::Node task_composer_plugins;
    if (!rhs.search_paths.empty())
      task_composer_plugins[SEARCH_PATHS_KEY] = rhs.search_paths;

    if (!rhs.search_libraries.empty())
      task_composer_plugins[SEARCH_LIBRARIES_KEY] = rhs.search_libraries;

    if (!rhs.executor_plugin_infos.plugins.empty())
      task_composer_plugins[EXECUTOR_PLUGINS_KEY] = rhs.executor_plugin_infos;

    if (!rhs.node_plugin_infos.plugins.empty())
      task_composer_plugins[NODE_PLUGINS_KEY] = rhs.node_plugin_infos;

    return task_composer_plugins;
  }

  static bool decode(const Node& node, tesseract_common::TaskComposerPluginInfo& rhs)
  {
    const std::string SEARCH_PATHS_KEY{ "search_paths" };
    const std::string SEARCH_LIBRARIES_KEY{ "search_libraries" };
    const std::string EXECUTOR_PLUGINS_KEY{ "executor_plugins" };
    const std::string NODE_PLUGINS_KEY{ "node_plugins" };

    if (const YAML::Node& search_paths = node[SEARCH_PATHS_KEY])
    {
      std::set<std::string> sp;
      try
      {
        sp = search_paths.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("TaskComposerPluginInfo: Constructor failed to cast '" + SEARCH_PATHS_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_paths.insert(sp.begin(), sp.end());
    }

    if (const YAML::Node& search_libraries = node[SEARCH_LIBRARIES_KEY])
    {
      std::set<std::string> sl;
      try
      {
        sl = search_libraries.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("TaskComposerPluginInfo: Constructor failed to cast '" + SEARCH_LIBRARIES_KEY +
                                 "' to std::set<std::string>! "
                                 "Details: " +
                                 e.what());
      }
      rhs.search_libraries.insert(sl.begin(), sl.end());
    }

    if (const YAML::Node& executor_plugins = node[EXECUTOR_PLUGINS_KEY])
    {
      if (!executor_plugins.IsMap())
        throw std::runtime_error(EXECUTOR_PLUGINS_KEY + ", should contain a map of task composer executor names to "
                                                        "plugins!");

      try
      {
        rhs.executor_plugin_infos = executor_plugins.as<tesseract_common::PluginInfoContainer>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("TaskComposerPluginInfo: Constructor failed to cast '" + EXECUTOR_PLUGINS_KEY +
                                 "' to tesseract_common::PluginInfoContainer! Details: " + e.what());
      }
    }

    if (const YAML::Node& node_plugins = node[NODE_PLUGINS_KEY])
    {
      if (!node_plugins.IsMap())
        throw std::runtime_error(NODE_PLUGINS_KEY + ", should contain a map of names to plugins!");

      try
      {
        rhs.node_plugin_infos = node_plugins.as<tesseract_common::PluginInfoContainer>();
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("TaskComposerPluginInfo: Constructor failed to cast '" + NODE_PLUGINS_KEY +
                                 "' to tesseract_common::PluginInfoContainer! Details: " + e.what());
      }
    }

    return true;
  }
};

template <>
struct convert<tesseract_common::TransformMap>
{
  static Node encode(const tesseract_common::TransformMap& rhs)
  {
    Node node;
    for (const auto& pair : rhs)
      node[pair.first] = pair.second;

    return node;
  }

  static bool decode(const Node& node, tesseract_common::TransformMap& rhs)
  {
    if (!node.IsMap())
      return false;

    for (const auto& pair : node)
      rhs[pair.first.as<std::string>()] = pair.second.as<Eigen::Isometry3d>();

    return true;
  }
};

template <>
struct convert<tesseract_common::CalibrationInfo>
{
  static Node encode(const tesseract_common::CalibrationInfo& rhs)
  {
    Node node;
    node["joints"] = rhs.joints;

    return node;
  }

  static bool decode(const Node& node, tesseract_common::CalibrationInfo& rhs)
  {
    const YAML::Node& joints_node = node["joints"];

    rhs.joints = joints_node.as<tesseract_common::TransformMap>();

    return true;
  }
};
}  // namespace YAML

#endif  // TESSERACT_COMMON_YAML_UTILS_H
