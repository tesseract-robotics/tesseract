/**
 * @file serialization.h
 * @brief Tesseracts Collision Serialization
 *
 * @author Levi Armstrong
 * @date March 20, 2023
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#ifndef TESSERACT_COLLISION_CEREAL_SERIALIZATION_H
#define TESSERACT_COLLISION_CEREAL_SERIALIZATION_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <cereal/cereal.hpp>
#include <cereal/types/atomic.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/common.hpp>
#include <cereal/types/polymorphic.hpp>
#include <tesseract/common/cereal_serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/types.h>
#include <tesseract/collision/contact_result_validator.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>

namespace tesseract::collision
{
/************************************************/
/****** tesseract::collision::ContactResult ******/
/************************************************/

template <class Archive>
void serialize(Archive& ar, tesseract::collision::ContactResult& g)
{
  ar(cereal::make_nvp("distance", g.distance));
  ar(cereal::make_nvp("type_id", g.type_id));
  // NVP key kept as "link_names" for archive compat with master; field was renamed to link_ids during Id migration.
  ar(cereal::make_nvp("link_names", g.link_ids));
  ar(cereal::make_nvp("shape_id", g.shape_id));
  ar(cereal::make_nvp("subshape_id", g.subshape_id));
  ar(cereal::make_nvp("nearest_points", g.nearest_points));
  ar(cereal::make_nvp("nearest_points_local", g.nearest_points_local));
  ar(cereal::make_nvp("transform", g.transform));
  ar(cereal::make_nvp("normal", g.normal));
  ar(cereal::make_nvp("cc_time", g.cc_time));
  ar(cereal::make_nvp("cc_type", g.cc_type));
  ar(cereal::make_nvp("cc_transform", g.cc_transform));
  ar(cereal::make_nvp("single_contact_point", g.single_contact_point));
}

/***(**********************************************/
/****** tesseract::collision::ContactResultMap ******/
/***************************************************/
template <class Archive>
void save(Archive& ar, const tesseract::collision::ContactResultMap& g)
{
  // Master-compatible wire format: sorted std::map keyed by canonical-ordered name pair under
  // NVP "container". The internal LinkIdPair key is reconstructed on load from each
  // ContactResult's link_ids[0/1] names (LinkId's save_minimal persists the original name).
  // OrderedIdPair has no cereal specialization on purpose — its NameIdValue ids are not stable
  // across builds, so we serialize the names instead.
  //
  // Note on canonicalization: we sort the (a,b) name pair here for deterministic output of *this*
  // branch's archives only. Master canonicalizes by call-site convention (insertion order from
  // the contact-test loop), so the resulting on-disk byte sequence is not guaranteed to match
  // master byte-for-byte — the wire *shape* (NVP "container", keyed string pair, ContactResult
  // value layout) is what matters for cross-build load compatibility.
  using KeyT = std::pair<std::string, std::string>;
  using MappedT = tesseract::collision::ContactResultVector;
  tesseract::common::AlignedMap<KeyT, MappedT> container;
  for (const auto& [key, results] : g.getContainer())
  {
    if (results.empty())
      continue;  // Master also dropped empty buckets; preserve that.
    const auto& names = results.front().link_ids;
    // Defensive guard against silent archive corruption: the wire key is derived from the stored
    // ContactResult's link_ids names. A default-constructed ContactResult (link_ids of
    // INVALID_LINK_ID) would silently emit ("","") as the key, dropping the original LinkIdPair
    // identity. The previous (LinkIdPair, results) format did not have this footgun, so callers
    // must populate link_ids before insertion.
    if (names[0].name().empty() || names[1].name().empty())
      throw std::runtime_error("ContactResultMap cereal save: stored ContactResult has empty/invalid "
                               "link_ids; cannot persist key — caller must populate link_ids before "
                               "insertion.");
    auto string_key = (names[0].name() <= names[1].name()) ? KeyT{ names[0].name(), names[1].name() } :
                                                             KeyT{ names[1].name(), names[0].name() };
    container.emplace(std::move(string_key), results);
  }
  ar(cereal::make_nvp("container", container));
}

template <class Archive>
void load(Archive& ar, tesseract::collision::ContactResultMap& g)
{
  using KeyT = std::pair<std::string, std::string>;
  using MappedT = tesseract::collision::ContactResultVector;
  tesseract::common::AlignedMap<KeyT, MappedT> container;
  ar(cereal::make_nvp("container", container));
  for (const auto& [string_key, results] : container)
  {
    tesseract::common::LinkIdPair pair_key(tesseract::common::LinkId(string_key.first),
                                           tesseract::common::LinkId(string_key.second));
    g.addContactResult(pair_key, results);
  }
}

template <class Archive>
void serialize(Archive& ar, tesseract::collision::ContactRequest& g)
{
  ar(cereal::make_nvp("type", g.type));
  ar(cereal::make_nvp("calculate_penetration", g.calculate_penetration));
  ar(cereal::make_nvp("calculate_distance", g.calculate_distance));
  ar(cereal::make_nvp("contact_limit", g.contact_limit));
  ar(cereal::make_nvp("is_valid", g.is_valid));
}

template <class Archive>
void save(Archive& ar, const tesseract::collision::ContactManagerConfig& g)
{
  ar(cereal::make_nvp("default_margin", g.default_margin));
  ar(cereal::make_nvp("pair_margin_override_type", g.pair_margin_override_type));
  ar(cereal::make_nvp("pair_margin_data", g.pair_margin_data));
  ar(cereal::make_nvp("acm", g.acm));
  ar(cereal::make_nvp("acm_override_type", g.acm_override_type));
  std::unordered_map<std::string, bool> moe;
  moe.reserve(g.modify_object_enabled.size());
  for (const auto& [id, val] : g.modify_object_enabled)
    moe[id.name()] = val;
  ar(cereal::make_nvp("modify_object_enabled", moe));
}

template <class Archive>
void load(Archive& ar, tesseract::collision::ContactManagerConfig& g)
{
  ar(cereal::make_nvp("default_margin", g.default_margin));
  ar(cereal::make_nvp("pair_margin_override_type", g.pair_margin_override_type));
  ar(cereal::make_nvp("pair_margin_data", g.pair_margin_data));
  ar(cereal::make_nvp("acm", g.acm));
  ar(cereal::make_nvp("acm_override_type", g.acm_override_type));
  std::unordered_map<std::string, bool> moe;
  ar(cereal::make_nvp("modify_object_enabled", moe));
  g.modify_object_enabled.clear();
  for (const auto& [name, val] : moe)
    g.modify_object_enabled[tesseract::common::LinkId(name)] = val;
}

template <class Archive>
void serialize(Archive& ar, tesseract::collision::CollisionCheckConfig& g)
{
  ar(cereal::make_nvp("contact_request", g.contact_request));
  ar(cereal::make_nvp("type", g.type));
  ar(cereal::make_nvp("longest_valid_segment_length", g.longest_valid_segment_length));
  ar(cereal::make_nvp("check_program_mode", g.check_program_mode));
  ar(cereal::make_nvp("exit_condition", g.exit_condition));
}

template <class Archive>
void serialize(Archive& ar, tesseract::collision::ContactTrajectorySubstepResults& g)
{
  ar(cereal::make_nvp("contacts", g.contacts));
  ar(cereal::make_nvp("substep", g.substep));
  ar(cereal::make_nvp("state0", g.state0));
  ar(cereal::make_nvp("state1", g.state1));
}

template <class Archive>
void serialize(Archive& ar, tesseract::collision::ContactTrajectoryStepResults& g)
{
  ar(cereal::make_nvp("substeps", g.substeps));
  ar(cereal::make_nvp("step", g.step));
  ar(cereal::make_nvp("state0", g.state0));
  ar(cereal::make_nvp("state1", g.state1));
  ar(cereal::make_nvp("total_substeps", g.total_substeps));
}

template <class Archive>
void serialize(Archive& ar, tesseract::collision::ContactTrajectoryResults& g)
{
  ar(cereal::make_nvp("steps", g.steps));
  // NVP key kept as "joint_names" for archive compat with master; field was renamed to joint_ids during Id migration.
  ar(cereal::make_nvp("joint_names", g.joint_ids));
  ar(cereal::make_nvp("total_steps", g.total_steps));
}
}  // namespace tesseract::collision

// On Windows the cereal polymorphic-type registration must be in the header,
// for other platforms registration is in the cpp.
#ifdef _WIN32
#include <tesseract/collision/cereal_serialization_impl.hpp>
#else
CEREAL_FORCE_DYNAMIC_INIT(tesseract_collision_cereal)
#endif

#endif  // TESSERACT_COLLISION_CEREAL_SERIALIZATION_H
