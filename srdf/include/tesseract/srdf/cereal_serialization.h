#ifndef TESSERACT_SRDF_CEREAL_SERIALIZATION_H
#define TESSERACT_SRDF_CEREAL_SERIALIZATION_H

#include <tesseract/srdf/kinematics_information.h>
#include <tesseract/srdf/srdf_model.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/unordered_map.hpp>

#include <tesseract/common/cereal_serialization.h>

namespace tesseract::srdf
{
template <class Archive>
void serialize(Archive& ar, KinematicsInformation& obj)
{
  ar(cereal::make_nvp("group_names", obj.group_names));
  ar(cereal::make_nvp("chain_groups", obj.chain_groups));
  ar(cereal::make_nvp("joint_groups", obj.joint_groups));
  ar(cereal::make_nvp("link_groups", obj.link_groups));
  ar(cereal::make_nvp("group_states", obj.group_states));
  ar(cereal::make_nvp("group_tcps", obj.group_tcps));
  ar(cereal::make_nvp("kinematics_plugin_info", obj.kinematics_plugin_info));
}

template <class Archive>
void serialize(Archive& ar, SRDFModel& obj)
{
  ar(cereal::make_nvp("name", obj.name));
  ar(cereal::make_nvp("version", obj.version));
  ar(cereal::make_nvp("kinematics_information", obj.kinematics_information));
  ar(cereal::make_nvp("contact_managers_plugin_info", obj.contact_managers_plugin_info));
  ar(cereal::make_nvp("acm", obj.acm));
  ar(cereal::make_nvp("collision_margin_data", obj.collision_margin_data));
  ar(cereal::make_nvp("calibration_info", obj.calibration_info));
}
}  // namespace tesseract::srdf

#endif  // TESSERACT_SRDF_CEREAL_SERIALIZATION_H
