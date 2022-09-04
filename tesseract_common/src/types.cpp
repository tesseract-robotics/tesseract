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
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/types.h>
#include <tesseract_common/yaml_utils.h>

namespace tesseract_common
{
std::size_t PairHash::operator()(const LinkNamesPair& pair) const
{
  return std::hash<std::string>()(pair.first + pair.second);
}

LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
{
  if (link_name1 <= link_name2)
    return std::make_pair(link_name1, link_name2);

  return std::make_pair(link_name2, link_name1);
}

/*********************************************************/
/******               PluginInfo                     *****/
/*********************************************************/

std::string PluginInfo::getConfigString() const
{
  std::stringstream stream;
  stream << config;
  return stream.str();
}

bool PluginInfo::operator==(const PluginInfo& rhs) const
{
  bool equal = true;
  equal &= class_name == rhs.class_name;
  equal &= isIdentical(config, rhs.config);

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
  config = YAML::Load(config_string);
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
  fwd_plugin_infos.insert(other.fwd_plugin_infos.begin(), other.fwd_plugin_infos.end());
  inv_plugin_infos.insert(other.inv_plugin_infos.begin(), other.inv_plugin_infos.end());
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
  equal &= fwd_plugin_infos == rhs.fwd_plugin_infos;
  equal &= inv_plugin_infos == rhs.inv_plugin_infos;

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
  discrete_plugin_infos.plugins.insert(other.discrete_plugin_infos.plugins.begin(),
                                       other.discrete_plugin_infos.plugins.end());
  continuous_plugin_infos.plugins.insert(other.continuous_plugin_infos.plugins.begin(),
                                         other.continuous_plugin_infos.plugins.end());

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
  equal &= discrete_plugin_infos == rhs.discrete_plugin_infos;
  equal &= continuous_plugin_infos == rhs.continuous_plugin_infos;

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
  executor_plugin_infos.plugins.insert(other.executor_plugin_infos.plugins.begin(),
                                       other.executor_plugin_infos.plugins.end());
  node_plugin_infos.plugins.insert(other.node_plugin_infos.plugins.begin(), other.node_plugin_infos.plugins.end());

  if (!other.executor_plugin_infos.default_plugin.empty())
    executor_plugin_infos.default_plugin = other.executor_plugin_infos.default_plugin;

  if (!other.node_plugin_infos.default_plugin.empty())
    node_plugin_infos.default_plugin = other.node_plugin_infos.default_plugin;
}

void TaskComposerPluginInfo::clear()
{
  search_paths.clear();
  search_libraries.clear();
  executor_plugin_infos.clear();
  node_plugin_infos.clear();
}

bool TaskComposerPluginInfo::empty() const
{
  return (search_paths.empty() && search_libraries.empty() && executor_plugin_infos.plugins.empty() &&
          node_plugin_infos.plugins.empty());
}

bool TaskComposerPluginInfo::operator==(const TaskComposerPluginInfo& rhs) const
{
  bool equal = true;
  equal &= isIdenticalSet<std::string>(search_paths, rhs.search_paths);
  equal &= isIdenticalSet<std::string>(search_libraries, rhs.search_libraries);
  equal &= executor_plugin_infos == rhs.executor_plugin_infos;
  equal &= node_plugin_infos == rhs.node_plugin_infos;

  return equal;
}
bool TaskComposerPluginInfo::operator!=(const TaskComposerPluginInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerPluginInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(search_paths);
  ar& BOOST_SERIALIZATION_NVP(search_libraries);
  ar& BOOST_SERIALIZATION_NVP(executor_plugin_infos);
  ar& BOOST_SERIALIZATION_NVP(node_plugin_infos);
}

/*********************************************************/
/******               CalibrationInfo                *****/
/*********************************************************/
void CalibrationInfo::insert(const CalibrationInfo& other) { joints.insert(other.joints.begin(), other.joints.end()); }

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
