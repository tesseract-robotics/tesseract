#ifndef TESSERACT_COMMON_CEREAL_SERIALIZATION_H
#define TESSERACT_COMMON_CEREAL_SERIALIZATION_H

#include <tesseract_common/any_poly.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/calibration_info.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/contact_allowed_validator.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/cereal_boost_types.h>
#include <tesseract_common/cereal_eigen_types.h>

#include <cereal/cereal.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/details/traits.hpp>

namespace tesseract_common
{
template <class Archive, class T>
void serialize(Archive& ar, AnyWrapper<T>& obj)
{
  ar(CEREAL_NVP(obj.value));
}

template <class Archive>
void serialize(Archive& ar, AnyPoly& obj)
{
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& ar, AllowedCollisionMatrix& obj)
{
  ar(cereal::make_nvp("lookup_table", obj.lookup_table_));
}

template <class Archive>
void serialize(Archive& ar, CalibrationInfo& obj)
{
  ar(CEREAL_NVP(obj.joints));
}

template <class Archive>
void serialize(Archive& ar, CollisionMarginPairData& obj)
{
  ar(cereal::make_nvp("lookup_table", obj.lookup_table_));

  // Recreate max_collision_margin_ and object_max_margins_ after deserialization
  if (Archive::is_loading::value)
  {
    obj.updateMaxMargins();
  }
}

template <class Archive>
void serialize(Archive& ar, CollisionMarginData& obj)
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
  ar(CEREAL_NVP(obj.joint_names));
  ar(CEREAL_NVP(obj.position));
  ar(CEREAL_NVP(obj.velocity));
  ar(CEREAL_NVP(obj.acceleration));
  ar(CEREAL_NVP(obj.effort));
  ar(CEREAL_NVP(obj.time));
}

template <class Archive>
void serialize(Archive& ar, JointTrajectory& obj)
{
  ar(CEREAL_NVP(obj.uuid));
  ar(CEREAL_NVP(obj.states));
  ar(CEREAL_NVP(obj.description));
}

template <class Archive>
void serialize(Archive& ar, ManipulatorInfo& obj)
{
  ar(CEREAL_NVP(obj.manipulator));
  ar(CEREAL_NVP(obj.manipulator_ik_solver));
  ar(CEREAL_NVP(obj.working_frame));
  ar(CEREAL_NVP(obj.tcp_frame));
  ar(CEREAL_NVP(obj.tcp_offset));
}

template <class Archive>
void serialize(Archive& ar, KinematicLimits& obj)
{
  ar(CEREAL_NVP(obj.joint_limits));
  ar(CEREAL_NVP(obj.velocity_limits));
  ar(CEREAL_NVP(obj.acceleration_limits));
  ar(CEREAL_NVP(obj.jerk_limits));
}

template <class Archive>
void serialize(Archive& /*ar*/, Resource& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& /*ar*/, GeneralResourceLocator& /*obj*/)
{
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
  ar(CEREAL_NVP(obj.class_name));
  const std::string config_string = obj.getConfigString();
  ar(CEREAL_NVP(config_string));
}

template <class Archive>
void load(Archive& ar, PluginInfo& obj)
{
  ar(CEREAL_NVP(obj.class_name));
  std::string config_string;
  ar(CEREAL_NVP(config_string));
  // On 18.04 '~' does not load as null so must check
  obj.config = (config_string != "~") ? YAML::Load(config_string) : YAML::Node();
}

template <class Archive>
void serialize(Archive& ar, PluginInfoContainer& obj)
{
  ar(CEREAL_NVP(obj.default_plugin));
  ar(CEREAL_NVP(obj.plugins));
}

template <class Archive>
void serialize(Archive& ar, ProfilesPluginInfo& obj)
{
  ar(CEREAL_NVP(obj.search_paths));
  ar(CEREAL_NVP(obj.search_libraries));
  ar(CEREAL_NVP(obj.plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, KinematicsPluginInfo& obj)
{
  ar(CEREAL_NVP(obj.search_paths));
  ar(CEREAL_NVP(obj.search_libraries));
  ar(CEREAL_NVP(obj.fwd_plugin_infos));
  ar(CEREAL_NVP(obj.inv_plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, ContactManagersPluginInfo& obj)
{
  ar(CEREAL_NVP(obj.search_paths));
  ar(CEREAL_NVP(obj.search_libraries));
  ar(CEREAL_NVP(obj.discrete_plugin_infos));
  ar(CEREAL_NVP(obj.continuous_plugin_infos));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerPluginInfo& obj)
{
  ar(CEREAL_NVP(obj.search_paths));
  ar(CEREAL_NVP(obj.search_libraries));
  ar(CEREAL_NVP(obj.executor_plugin_infos));
  ar(CEREAL_NVP(obj.task_plugin_infos));
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

}  // namespace tesseract_common

CEREAL_REGISTER_TYPE(tesseract_common::BoolAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::IntAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UnsignedAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::DoubleAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::FloatAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::StringAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::SizeTAnyPoly)

CEREAL_REGISTER_TYPE(tesseract_common::VectorStringAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorBoolAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorIntAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorUnsignedAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorDoubleAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorFloatAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::VectorSizeTAnyPoly)

CEREAL_REGISTER_TYPE(tesseract_common::UMapStringStringAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringBoolAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringIntAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringUnsignedAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringDoubleAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringFloatAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::UMapStringSizeTAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::BoolAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::IntAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UnsignedAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::DoubleAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::FloatAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::StringAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::SizeTAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorStringAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorBoolAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorIntAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorUnsignedAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorDoubleAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorFloatAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::VectorSizeTAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringStringAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringBoolAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringIntAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringUnsignedAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringDoubleAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringFloatAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::UMapStringSizeTAnyPoly)

CEREAL_REGISTER_TYPE(tesseract_common::ACMContactAllowedValidator)
CEREAL_REGISTER_TYPE(tesseract_common::CombinedContactAllowedValidator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::ContactAllowedValidator,
                                     tesseract_common::ACMContactAllowedValidator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::ContactAllowedValidator,
                                     tesseract_common::CombinedContactAllowedValidator)

CEREAL_REGISTER_TYPE(tesseract_common::JointStateAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_common::JointStatePtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::JointStateAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::JointStatePtrAnyPoly)

CEREAL_REGISTER_TYPE(tesseract_common::ManipulatorInfoAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::ManipulatorInfoAnyPoly)

CEREAL_REGISTER_TYPE(tesseract_common::Resource)
CEREAL_REGISTER_TYPE(tesseract_common::GeneralResourceLocator)
CEREAL_REGISTER_TYPE(tesseract_common::SimpleLocatedResource)
CEREAL_REGISTER_TYPE(tesseract_common::BytesResource)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::ResourceLocator, tesseract_common::Resource)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::ResourceLocator, tesseract_common::GeneralResourceLocator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Resource, tesseract_common::SimpleLocatedResource)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Resource, tesseract_common::BytesResource)

CEREAL_REGISTER_TYPE(tesseract_common::ProfileDictionaryPtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_common::ProfileDictionaryPtrAnyPoly)

#endif  // TESSERACT_COMMON_CEREAL_SERIALIZATION_H
