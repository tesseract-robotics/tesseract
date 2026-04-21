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
  // Serialize as vector of (LinkIdPair, results) to avoid unordered_map serialization
  std::vector<std::pair<tesseract::common::LinkIdPair, tesseract::collision::ContactResultVector>> entries;
  for (const auto& [key, results] : g.getContainer())
  {
    if (!results.empty())
      entries.emplace_back(key, results);
  }
  // Sort by LinkIdPair for deterministic output
  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  ar(cereal::make_nvp("entries", entries));
}

template <class Archive>
void load(Archive& ar, tesseract::collision::ContactResultMap& g)
{
  std::vector<std::pair<tesseract::common::LinkIdPair, tesseract::collision::ContactResultVector>> entries;
  ar(cereal::make_nvp("entries", entries));

  for (const auto& [key, results] : entries)
    g.addContactResult(key, results);
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
  ar(cereal::make_nvp("joint_names", g.joint_ids));
  ar(cereal::make_nvp("total_steps", g.total_steps));
}
}  // namespace tesseract::collision

#include <tesseract/collision/cereal_serialization_impl.hpp>

#endif  // TESSERACT_COLLISION_CEREAL_SERIALIZATION_H
