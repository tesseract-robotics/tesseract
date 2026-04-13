#ifndef TESSERACT_COMMON_CEREAL_SERIALIZATION_H
#define TESSERACT_COMMON_CEREAL_SERIALIZATION_H

#include <tesseract/common/any_poly.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/calibration_info.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/common/contact_allowed_validator.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/types.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/cereal_boost_types.h>
#include <tesseract/common/cereal_eigen_types.h>

#include <cereal/cereal.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/details/traits.hpp>

namespace tesseract::common
{
template <class Archive, typename Tag>
std::string save_minimal(const Archive&, const NameId<Tag>& id)
{
  return id.name();
}

template <class Archive, typename Tag>
void load_minimal(const Archive&, NameId<Tag>& id, const std::string& value)
{
  id = NameId<Tag>::fromName(value);
}

template <class Archive>
void serialize(Archive& ar, LinkIdPair& pair)
{
  ar(cereal::make_nvp("first", pair.first));
  ar(cereal::make_nvp("second", pair.second));
}

template <class Archive, class T>
void serialize(Archive& ar, AnyWrapper<T>& obj)
{
  ar(cereal::make_nvp("value", obj.value));
}

template <class Archive>
void serialize(Archive& ar, AnyPoly& obj)
{
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void save(Archive& ar, const AllowedCollisionMatrix& obj)
{
  // Serialize as string-based format for backwards compatibility
  std::map<std::pair<std::string, std::string>, std::string> compat;
  for (const auto& [key, entry] : obj.lookup_table_)
    compat[{ entry.name1, entry.name2 }] = entry.reason;
  ar(cereal::make_nvp("lookup_table", compat));
}

template <class Archive>
void load(Archive& ar, AllowedCollisionMatrix& obj)
{
  std::map<std::pair<std::string, std::string>, std::string> compat;
  ar(cereal::make_nvp("lookup_table", compat));
  for (const auto& [names, reason] : compat)
    obj.addAllowedCollision(names.first, names.second, reason);
}

template <class Archive>
void serialize(Archive& ar, CalibrationInfo& obj)
{
  ar(cereal::make_nvp("joints", obj.joints));
}

template <class Archive>
void save(Archive& ar, const CollisionMarginPairData& obj)
{
  // Serialize as string-based format for backwards compatibility
  std::map<std::pair<std::string, std::string>, double> compat;
  for (const auto& [key, margin] : obj.lookup_table_)
    compat[{ key.first.name(), key.second.name() }] = margin;
  ar(cereal::make_nvp("lookup_table", compat));
}

template <class Archive>
void load(Archive& ar, CollisionMarginPairData& obj)
{
  std::map<std::pair<std::string, std::string>, double> compat;
  ar(cereal::make_nvp("lookup_table", compat));
  for (const auto& [names, margin] : compat)
    obj.setCollisionMargin(names.first, names.second, margin);
}

template <class Archive>
void save(Archive& ar, const CollisionMarginData& obj)
{
  ar(cereal::make_nvp("default_collision_margin", obj.default_collision_margin_));
  ar(cereal::make_nvp("pair_margins", obj.pair_margins_));
}

template <class Archive>
void load(Archive& ar, CollisionMarginData& obj)
{
  ar(cereal::make_nvp("default_collision_margin", obj.default_collision_margin_));
  ar(cereal::make_nvp("pair_margins", obj.pair_margins_));
}

template <class Archive>
void serialize(Archive& ar, ACMContactAllowedValidator& obj)
{
  ar(cereal::make_nvp("acm", obj.acm_));
}

template <class Archive>
void serialize(Archive& ar, CombinedContactAllowedValidator& obj)
{
  ar(cereal::make_nvp("validators", obj.validators_));
  ar(cereal::make_nvp("type", obj.type_));
}

template <class Archive>
void serialize(Archive& ar, JointState& obj)
{
  ar(cereal::make_nvp("joint_names", obj.joint_ids));
  ar(cereal::make_nvp("position", obj.position));
  ar(cereal::make_nvp("velocity", obj.velocity));
  ar(cereal::make_nvp("acceleration", obj.acceleration));
  ar(cereal::make_nvp("effort", obj.effort));
  ar(cereal::make_nvp("time", obj.time));
}

template <class Archive>
void serialize(Archive& ar, JointTrajectory& obj)
{
  ar(cereal::make_nvp("uuid", obj.uuid));
  ar(cereal::make_nvp("states", obj.states));
  ar(cereal::make_nvp("description", obj.description));
}

template <class Archive>
void save(Archive& ar, const ManipulatorInfo& obj)
{
  ar(cereal::make_nvp("manipulator", obj.manipulator));
  ar(cereal::make_nvp("manipulator_ik_solver", obj.manipulator_ik_solver));
  ar(cereal::make_nvp("working_frame", obj.working_frame));
  ar(cereal::make_nvp("tcp_frame", obj.tcp_frame));

  // Serialize tcp_offset as variant<string, Isometry3d> for backward compat
  if (obj.tcp_offset.index() == 0)
  {
    std::variant<std::string, Eigen::Isometry3d> tcp_offset_str(std::get<LinkId>(obj.tcp_offset).name());
    ar(cereal::make_nvp("tcp_offset", tcp_offset_str));
  }
  else
  {
    std::variant<std::string, Eigen::Isometry3d> tcp_offset_tf(std::get<Eigen::Isometry3d>(obj.tcp_offset));
    ar(cereal::make_nvp("tcp_offset", tcp_offset_tf));
  }
}

template <class Archive>
void load(Archive& ar, ManipulatorInfo& obj)
{
  ar(cereal::make_nvp("manipulator", obj.manipulator));
  ar(cereal::make_nvp("manipulator_ik_solver", obj.manipulator_ik_solver));
  ar(cereal::make_nvp("working_frame", obj.working_frame));
  ar(cereal::make_nvp("tcp_frame", obj.tcp_frame));

  // Load tcp_offset as variant<string, Isometry3d>, then convert string to LinkId
  std::variant<std::string, Eigen::Isometry3d> tcp_offset_compat;
  ar(cereal::make_nvp("tcp_offset", tcp_offset_compat));
  if (tcp_offset_compat.index() == 0)
    obj.tcp_offset = LinkId::fromName(std::get<std::string>(tcp_offset_compat));
  else
    obj.tcp_offset = std::get<Eigen::Isometry3d>(tcp_offset_compat);
}

template <class Archive>
void serialize(Archive& ar, KinematicLimits& obj)
{
  ar(cereal::make_nvp("joint_limits", obj.joint_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("jerk_limits", obj.jerk_limits));
}

template <class Archive>
void serialize(Archive& /*ar*/, Resource& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& /*ar*/, GeneralResourceLocator& /*obj*/)
{
  // This should not be serialized and dynamically loaded to pull host paths
  // ar(cereal::make_nvp("package_paths", obj.package_paths_));
}

template <class Archive>
void serialize(Archive& ar, SimpleLocatedResource& obj)
{
  ar(cereal::make_nvp("url", obj.url_));
  ar(cereal::make_nvp("filename", obj.filename_));
  ar(cereal::make_nvp("parent", obj.parent_));
}

template <class Archive>
void serialize(Archive& ar, BytesResource& obj)
{
  ar(cereal::make_nvp("url", obj.url_));
  ar(cereal::make_nvp("bytes", obj.bytes_));
  ar(cereal::make_nvp("parent", obj.parent_));
}

template <class Archive>
void save(Archive& ar, const PluginInfo& obj)
{
  ar(cereal::make_nvp("class_name", obj.class_name));
  std::string config_string = obj.getConfigString();
  ar(cereal::make_nvp("config", config_string));
}

template <class Archive>
void load(Archive& ar, PluginInfo& obj)
{
  ar(cereal::make_nvp("class_name", obj.class_name));
  std::string config_string;
  ar(cereal::make_nvp("config", config_string));
  // On 18.04 '~' does not load as null so must check
  obj.config = (config_string != "~") ? YAML::Load(config_string) : YAML::Node();
}

template <class Archive>
void serialize(Archive& ar, PluginInfoContainer& obj)
{
  ar(cereal::make_nvp("default_plugin", obj.default_plugin));
  ar(cereal::make_nvp("plugins", obj.plugins));
}

template <class Archive>
void serialize(Archive& ar, ProfilesPluginInfo& obj)
{
  ar(cereal::make_nvp("search_paths", obj.search_paths));
  ar(cereal::make_nvp("search_libraries", obj.search_libraries));
  ar(cereal::make_nvp("plugin_infos", obj.plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, KinematicsPluginInfo& obj)
{
  ar(cereal::make_nvp("search_paths", obj.search_paths));
  ar(cereal::make_nvp("search_libraries", obj.search_libraries));
  ar(cereal::make_nvp("fwd_plugin_infos", obj.fwd_plugin_infos));
  ar(cereal::make_nvp("inv_plugin_infos", obj.inv_plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, ContactManagersPluginInfo& obj)
{
  ar(cereal::make_nvp("search_paths", obj.search_paths));
  ar(cereal::make_nvp("search_libraries", obj.search_libraries));
  ar(cereal::make_nvp("discrete_plugin_infos", obj.discrete_plugin_infos));
  ar(cereal::make_nvp("continuous_plugin_infos", obj.continuous_plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerPluginInfo& obj)
{
  ar(cereal::make_nvp("search_paths", obj.search_paths));
  ar(cereal::make_nvp("search_libraries", obj.search_libraries));
  ar(cereal::make_nvp("executor_plugin_infos", obj.executor_plugin_infos));
  ar(cereal::make_nvp("task_plugin_infos", obj.task_plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, Profile& obj)
{
  ar(cereal::make_nvp("key", obj.key_));
}

template <class Archive>
void serialize(Archive& ar, ProfileDictionary& obj)
{
  const std::shared_lock lock(obj.mutex_);
  ar(cereal::make_nvp("profiles", obj.profiles_));
}

}  // namespace tesseract::common

#include <tesseract/common/cereal_serialization_impl.hpp>

#endif  // TESSERACT_COMMON_CEREAL_SERIALIZATION_H
