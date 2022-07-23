/**
 * @file types.h
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_TYPES_H
#define TESSERACT_COMMON_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/kinematic_limits.h>

namespace tesseract_common
{
/** @brief Enable easy switching to std::filesystem when available */
namespace fs = boost::filesystem;

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename Key, typename Value>
using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key,
                                               Value,
                                               std::hash<Key>,
                                               std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
using VectorVector4d = AlignedVector<Eigen::Vector4d>;
using VectorVector3d = std::vector<Eigen::Vector3d>;
using VectorVector2d = AlignedVector<Eigen::Vector2d>;
using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;
using Toolpath = AlignedVector<VectorIsometry3d>;

using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using LinkNamesPair = std::pair<std::string, std::string>;

struct PairHash
{
  std::size_t operator()(const LinkNamesPair& pair) const;
};

/**
 * @brief Create a pair of strings, where the pair.first is always <= pair.second.
 *
 * This is commonly used along with PairHash as the key to an unordered_map<LinkNamesPair, Type, PairHash>
 * @param link_name1 First link name
 * @param link_name2 Second link name
 * @return LinkNamesPair a lexicographically sorted pair of strings
 */
LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2);

/** @brief The Plugin Information struct */
// NOLINTNEXTLINE
struct PluginInfo
{
  /** @brief The plugin class name */
  std::string class_name;

  /** @brief The plugin config data */
  YAML::Node config;

  /** @brief Get the yaml config as a string */
  std::string getConfigString() const;

  bool operator==(const PluginInfo& rhs) const;
  bool operator!=(const PluginInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void save(Archive& ar, const unsigned int version) const;  // NOLINT

  template <class Archive>
  void load(Archive& ar, const unsigned int version);  // NOLINT

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief A map of PluginInfo to user defined name */
using PluginInfoMap = std::map<std::string, PluginInfo>;

struct PluginInfoContainer
{
  std::string default_plugin;
  PluginInfoMap plugins;
  void clear();

  bool operator==(const PluginInfoContainer& rhs) const;
  bool operator!=(const PluginInfoContainer& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief The kinematics plugin information structure */
struct KinematicsPluginInfo
{
  /** @brief A list of paths to search for plugins */
  std::set<std::string> search_paths;

  /** @brief A list of library names without the prefix or suffix that contain plugins*/
  std::set<std::string> search_libraries;

  /** @brief A map of group name to forward kinematics plugin information */
  std::map<std::string, tesseract_common::PluginInfoContainer> fwd_plugin_infos;

  /** @brief A map of group name to inverse kinematics plugin information */
  std::map<std::string, tesseract_common::PluginInfoContainer> inv_plugin_infos;

  /** @brief Insert the content of an other KinematicsPluginInfo */
  void insert(const KinematicsPluginInfo& other);

  /** @brief Clear the contents */
  void clear();

  /** @brief Check if structure is empty */
  bool empty() const;

  // Yaml Config key
  static inline const std::string CONFIG_KEY{ "kinematic_plugins" };

  bool operator==(const KinematicsPluginInfo& rhs) const;
  bool operator!=(const KinematicsPluginInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief The contact managers plugin information structure */
struct ContactManagersPluginInfo
{
  /** @brief A list of paths to search for plugins */
  std::set<std::string> search_paths;

  /** @brief A list of library names without the prefix or suffix that contain plugins*/
  std::set<std::string> search_libraries;

  /** @brief A map of name to discrete contact manager plugin information */
  tesseract_common::PluginInfoContainer discrete_plugin_infos;

  /** @brief A map of name to continuous contact manager plugin information */
  tesseract_common::PluginInfoContainer continuous_plugin_infos;

  /** @brief Insert the content of an other ContactManagersPluginInfo */
  void insert(const ContactManagersPluginInfo& other);

  /** @brief Clear the contents */
  void clear();

  /** @brief Check if structure is empty */
  bool empty() const;

  // Yaml Config key
  static inline const std::string CONFIG_KEY{ "contact_manager_plugins" };

  bool operator==(const ContactManagersPluginInfo& rhs) const;
  bool operator!=(const ContactManagersPluginInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief The CalibrationInfo struct */
struct CalibrationInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CalibrationInfo() = default;

  /**
   *  @brief The joint origin calibration information
   *  @details For each entry in \p joints the environment will apply a ChangeJointOriginCommand replacing the current
   * joint origin with what is stored in the TransformMap
   */
  tesseract_common::TransformMap joints;

  /** @brief Insert the content of an other CalibrationInfo */
  void insert(const CalibrationInfo& other);

  /** @brief Clear the contents */
  void clear();

  /** @brief Check if structure is empty */
  bool empty() const;

  // Yaml Config key
  static inline const std::string CONFIG_KEY{ "calibration" };

  bool operator==(const CalibrationInfo& rhs) const;
  bool operator!=(const CalibrationInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_common

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_common::PluginInfo, "PluginInfo")
BOOST_CLASS_EXPORT_KEY2(tesseract_common::PluginInfoContainer, "PluginInfoContainer")
BOOST_CLASS_EXPORT_KEY2(tesseract_common::KinematicsPluginInfo, "KinematicsPluginInfo")
BOOST_CLASS_EXPORT_KEY2(tesseract_common::ContactManagersPluginInfo, "ContactManagersPluginInfo")
BOOST_CLASS_EXPORT_KEY2(tesseract_common::CalibrationInfo, "CalibrationInfo")

#endif  // TESSERACT_COMMON_TYPES_H
