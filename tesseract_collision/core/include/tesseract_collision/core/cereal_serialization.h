/**
 * @file serialization.h
 * @brief Tesseracts Collision Serialization
 *
 * @author Levi Armstrong
 * @date March 20, 2023
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cereal/cereal.hpp>
#include <cereal/types/atomic.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/polymorphic.hpp>
#include <tesseract_common/cereal_eigen_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{
/************************************************/
/****** tesseract_collision::ContactResult ******/
/************************************************/

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactResult& g)
{
  ar(cereal::make_nvp("distance", g.distance));
  ar(cereal::make_nvp("type_id", g.type_id));
  ar(cereal::make_nvp("link_names", g.link_names));
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
/****** tesseract_collision::ContactResultMap ******/
/***************************************************/
template <class Archive>
void save(Archive& ar, const tesseract_collision::ContactResultMap& g)
{
  ar(cereal::make_nvp("container", g.getContainer()));
}

template <class Archive>
void load(Archive& ar, tesseract_collision::ContactResultMap& g)
{
  tesseract_collision::ContactResultMap::ContainerType container;
  ar(cereal::make_nvp("container", container));

  for (const auto& c : container)
    g.addContactResult(c.first, c.second);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactRequest& g)
{
  ar(cereal::make_nvp("type", g.type));
  ar(cereal::make_nvp("calculate_penetration", g.calculate_penetration));
  ar(cereal::make_nvp("calculate_distance", g.calculate_distance));
  ar(cereal::make_nvp("contact_limit", g.contact_limit));
  ar(cereal::make_nvp("is_valid", g.is_valid));
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactManagerConfig& g)
{
  ar(cereal::make_nvp("default_margin", g.default_margin));
  ar(cereal::make_nvp("pair_margin_override_type", g.pair_margin_override_type));
  ar(cereal::make_nvp("pair_margin_data", g.pair_margin_data));
  ar(cereal::make_nvp("acm", g.acm));
  ar(cereal::make_nvp("acm_override_type", g.acm_override_type));
  ar(cereal::make_nvp("modify_object_enabled", g.modify_object_enabled));
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::CollisionCheckConfig& g)
{
  ar(cereal::make_nvp("contact_request", g.contact_request));
  ar(cereal::make_nvp("type", g.type));
  ar(cereal::make_nvp("longest_valid_segment_length", g.longest_valid_segment_length));
  ar(cereal::make_nvp("check_program_mode", g.check_program_mode));
  ar(cereal::make_nvp("exit_condition", g.exit_condition));
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectorySubstepResults& g)
{
  ar(cereal::make_nvp("contacts", g.contacts));
  ar(cereal::make_nvp("substep", g.substep));
  ar(cereal::make_nvp("state0", g.state0));
  ar(cereal::make_nvp("state1", g.state1));
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectoryStepResults& g)
{
  ar(cereal::make_nvp("substeps", g.substeps));
  ar(cereal::make_nvp("step", g.step));
  ar(cereal::make_nvp("state0", g.state0));
  ar(cereal::make_nvp("state1", g.state1));
  ar(cereal::make_nvp("total_substeps", g.total_substeps));
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectoryResults& g)
{
  ar(cereal::make_nvp("steps", g.steps));
  ar(cereal::make_nvp("joint_names", g.joint_names));
  ar(cereal::make_nvp("total_steps", g.total_steps));
}
}  // namespace tesseract_collision

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultMapVectorAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultMapVectorAnyPoly)

#endif  // TESSERACT_COLLISION_CEREAL_SERIALIZATION_H
