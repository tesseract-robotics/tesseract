/**
 * @file types.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/types.h>
#include <tesseract_common/yaml_utils.h>

namespace tesseract_common
{
std::size_t PairHash::operator()(const LinkNamesPair& pair) const
{
  std::size_t seed{ 0 };
  boost::hash_combine(seed, pair.first);
  boost::hash_combine(seed, pair.second);
  return seed;
}

LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
{
  return (link_name1 <= link_name2) ? std::make_pair(link_name1, link_name2) : std::make_pair(link_name2, link_name1);
}

void makeOrderedLinkPair(LinkNamesPair& pair, const std::string& link_name1, const std::string& link_name2)
{
  if (link_name1 <= link_name2)
  {
    pair.first = link_name1;
    pair.second = link_name2;
  }
  else
  {
    pair.first = link_name2;
    pair.second = link_name1;
  }
}

/*********************************************************/
/******               PluginInfo                     *****/
/*********************************************************/

std::string PluginInfo::getConfigString() const { return toYAMLString(config); }

bool PluginInfo::operator==(const PluginInfo& rhs) const
{
  bool equal = true;
  equal &= class_name == rhs.class_name;
  equal &= compareYAML(config, rhs.config);

  return equal;
}
bool PluginInfo::operator!=(const PluginInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void PluginInfo::save(Archive& ar, const unsigned int /*version*/) const
{
  ar& BOOST_SERIALIZATION_NVP(class_name);
  std::string config_string = getConfigString();
  ar& BOOST_SERIALIZATION_NVP(config_string);
}

template <class Archive>
void PluginInfo::load(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(class_name);
  std::string config_string;
  ar& BOOST_SERIALIZATION_NVP(config_string);
  // On 18.04 '~' does not load as null so must check
  config = (config_string != "~") ? YAML::Load(config_string) : YAML::Node();
}

template <class Archive>
void PluginInfo::serialize(Archive& ar, const unsigned int version)
{
  boost::serialization::split_member(ar, *this, version);
}

/*********************************************************/
/******           PluginInfoContainer                *****/
/*********************************************************/

void PluginInfoContainer::clear()
{
  default_plugin.clear();
  plugins.clear();
}

bool PluginInfoContainer::operator==(const PluginInfoContainer& rhs) const
{
  bool equal = true;
  equal &= default_plugin == rhs.default_plugin;
  equal &= isIdenticalMap<PluginInfoMap, PluginInfo>(plugins, rhs.plugins);

  return equal;
}
bool PluginInfoContainer::operator!=(const PluginInfoContainer& rhs) const { return !operator==(rhs); }

template <class Archive>
void PluginInfoContainer::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(default_plugin);
  ar& BOOST_SERIALIZATION_NVP(plugins);
}

/*********************************************************/
/******           KinematicsPluginInfo               *****/
/*********************************************************/
void KinematicsPluginInfo::insert(const KinematicsPluginInfo& other)
{
  search_paths.insert(other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(other.search_libraries.begin(), other.search_libraries.end());

  for (const auto& group_plugins : other.fwd_plugin_infos)
  {
    if (!group_plugins.second.default_plugin.empty())
      fwd_plugin_infos[group_plugins.first].default_plugin = group_plugins.second.default_plugin;

    for (const auto& plugin_info : group_plugins.second.plugins)
      fwd_plugin_infos[group_plugins.first].plugins[plugin_info.first] = plugin_info.second;
  }

  for (const auto& group_plugins : other.inv_plugin_infos)
  {
    if (!group_plugins.second.default_plugin.empty())
      inv_plugin_infos[group_plugins.first].default_plugin = group_plugins.second.default_plugin;

    for (const auto& plugin_info : group_plugins.second.plugins)
      inv_plugin_infos[group_plugins.first].plugins[plugin_info.first] = plugin_info.second;
  }
}

void KinematicsPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  fwd_plugin_infos.clear();
  inv_plugin_infos.clear();
}

bool KinematicsPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && fwd_plugin_infos.empty() && inv_plugin_infos.empty());
}

bool KinematicsPluginInfo::operator==(const KinematicsPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdenticalSet<std::string>(search_paths, rhs.search_paths);
  equal &= isIdenticalSet<std::string>(search_libraries, rhs.search_libraries);
  equal &= isIdenticalMap<std::map<std::string, PluginInfoContainer>, PluginInfoContainer>(fwd_plugin_infos,
                                                                                           rhs.fwd_plugin_infos);
  equal &= isIdenticalMap<std::map<std::string, PluginInfoContainer>, PluginInfoContainer>(inv_plugin_infos,
                                                                                           rhs.inv_plugin_infos);

  return equal;
}
bool KinematicsPluginInfo::operator!=(const KinematicsPluginInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void KinematicsPluginInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(search_paths);
  ar& BOOST_SERIALIZATION_NVP(search_libraries);
  ar& BOOST_SERIALIZATION_NVP(fwd_plugin_infos);
  ar& BOOST_SERIALIZATION_NVP(inv_plugin_infos);
}

/*********************************************************/
/******          ContactManagersPluginInfo           *****/
/*********************************************************/
void ContactManagersPluginInfo::insert(const ContactManagersPluginInfo& other)
{
  search_paths.insert(other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(other.search_libraries.begin(), other.search_libraries.end());

  if (!other.discrete_plugin_infos.default_plugin.empty())
    discrete_plugin_infos.default_plugin = other.discrete_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.discrete_plugin_infos.plugins)
    discrete_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.continuous_plugin_infos.default_plugin.empty())
    continuous_plugin_infos.default_plugin = other.continuous_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.continuous_plugin_infos.plugins)
    continuous_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.discrete_plugin_infos.default_plugin.empty())
    discrete_plugin_infos.default_plugin = other.discrete_plugin_infos.default_plugin;

  if (!other.continuous_plugin_infos.default_plugin.empty())
    continuous_plugin_infos.default_plugin = other.continuous_plugin_infos.default_plugin;
}

void ContactManagersPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  discrete_plugin_infos.clear();
  continuous_plugin_infos.clear();
}

bool ContactManagersPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && discrete_plugin_infos.plugins.empty() &&
          continuous_plugin_infos.plugins.empty());
}

bool ContactManagersPluginInfo::operator==(const ContactManagersPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdenticalSet<std::string>(search_paths, rhs.search_paths);
  equal &= isIdenticalSet<std::string>(search_libraries, rhs.search_libraries);
  equal &= (discrete_plugin_infos == rhs.discrete_plugin_infos);
  equal &= (continuous_plugin_infos == rhs.continuous_plugin_infos);
  return equal;
}
bool ContactManagersPluginInfo::operator!=(const ContactManagersPluginInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void ContactManagersPluginInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(search_paths);
  ar& BOOST_SERIALIZATION_NVP(search_libraries);
  ar& BOOST_SERIALIZATION_NVP(discrete_plugin_infos);
  ar& BOOST_SERIALIZATION_NVP(continuous_plugin_infos);
}

/*********************************************************/
/******          TaskComposerPluginInfo           *****/
/*********************************************************/
void TaskComposerPluginInfo::insert(const TaskComposerPluginInfo& other)
{
  search_paths.insert(other.search_paths.begin(), other.search_paths.end());
  search_libraries.insert(other.search_libraries.begin(), other.search_libraries.end());

  if (!other.executor_plugin_infos.default_plugin.empty())
    executor_plugin_infos.default_plugin = other.executor_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.executor_plugin_infos.plugins)
    executor_plugin_infos.plugins[plugin_info.first] = plugin_info.second;

  if (!other.task_plugin_infos.default_plugin.empty())
    task_plugin_infos.default_plugin = other.task_plugin_infos.default_plugin;

  for (const auto& plugin_info : other.task_plugin_infos.plugins)
    task_plugin_infos.plugins[plugin_info.first] = plugin_info.second;
}

void TaskComposerPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  executor_plugin_infos.clear();
  task_plugin_infos.clear();
}

bool TaskComposerPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && executor_plugin_infos.plugins.empty() &&
          task_plugin_infos.plugins.empty());
}

bool TaskComposerPluginInfo::operator==(const TaskComposerPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdenticalSet<std::string>(search_paths, rhs.search_paths);
  equal &= isIdenticalSet<std::string>(search_libraries, rhs.search_libraries);
  equal &= (executor_plugin_infos == rhs.executor_plugin_infos);
  equal &= (task_plugin_infos == rhs.task_plugin_infos);
  return equal;
}
bool TaskComposerPluginInfo::operator!=(const TaskComposerPluginInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerPluginInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(search_paths);
  ar& BOOST_SERIALIZATION_NVP(search_libraries);
  ar& BOOST_SERIALIZATION_NVP(executor_plugin_infos);
  ar& BOOST_SERIALIZATION_NVP(task_plugin_infos);
}

/*********************************************************/
/******               CalibrationInfo                *****/
/*********************************************************/
void CalibrationInfo::insert(const CalibrationInfo& other)
{
  for (const auto& joint : other.joints)
    joints[joint.first] = joint.second;
}

void CalibrationInfo::clear() { joints.clear(); }

bool CalibrationInfo::empty() const { return joints.empty(); }

bool CalibrationInfo::operator==(const CalibrationInfo& rhs) const
{
  auto isometry_equal = [](const Eigen::Isometry3d& iso_1, const Eigen::Isometry3d& iso_2) {
    return iso_1.isApprox(iso_2, 1e-5);
  };

  bool equal = true;
  equal &= tesseract_common::isIdenticalMap<TransformMap, Eigen::Isometry3d>(joints, rhs.joints, isometry_equal);

  return equal;
}
bool CalibrationInfo::operator!=(const CalibrationInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void CalibrationInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(joints);
}
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::PluginInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::PluginInfo)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::PluginInfoContainer)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::PluginInfoContainer)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::KinematicsPluginInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::KinematicsPluginInfo)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ContactManagersPluginInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ContactManagersPluginInfo)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::TaskComposerPluginInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::TaskComposerPluginInfo)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::CalibrationInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::CalibrationInfo)
