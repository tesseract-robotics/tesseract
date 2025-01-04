/**
 * @file serialization.cpp
 * @brief Contact results serialization wrappers
 * @details Supports the following
 *            - tesseract_collision::ContactResult
 *            - tesseract_collision::ContactResultVector
 *            - tesseract_collision::ContactResultMap
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <tesseract_common/eigen_serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/serialization.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/contact_result_validator.h>

namespace boost::serialization
{
/************************************************/
/****** tesseract_collision::ContactResult ******/
/************************************************/

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactResult& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("distance", g.distance);
  ar& boost::serialization::make_nvp("type_id", g.type_id);
  ar& boost::serialization::make_nvp("link_names", g.link_names);
  ar& boost::serialization::make_nvp("shape_id", g.shape_id);
  ar& boost::serialization::make_nvp("subshape_id", g.subshape_id);
  ar& boost::serialization::make_nvp("nearest_points", g.nearest_points);
  ar& boost::serialization::make_nvp("nearest_points_local", g.nearest_points_local);
  ar& boost::serialization::make_nvp("transform", g.transform);
  ar& boost::serialization::make_nvp("normal", g.normal);
  ar& boost::serialization::make_nvp("cc_time", g.cc_time);
  ar& boost::serialization::make_nvp("cc_type", g.cc_type);
  ar& boost::serialization::make_nvp("cc_transform", g.cc_transform);
  ar& boost::serialization::make_nvp("single_contact_point", g.single_contact_point);
}

/***************************************************/
/****** tesseract_collision::ContactResultMap ******/
/***************************************************/
template <class Archive>
void save(Archive& ar, const tesseract_collision::ContactResultMap& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("container", g.getContainer());
}

template <class Archive>
void load(Archive& ar, tesseract_collision::ContactResultMap& g, const unsigned int /*version*/)
{
  tesseract_collision::ContactResultMap::ContainerType container;
  ar& boost::serialization::make_nvp("container", container);

  for (const auto& c : container)
    g.addContactResult(c.first, c.second);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactResultMap& g, const unsigned int version)
{
  split_free(ar, g, version);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactRequest& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("type", g.type);
  ar& boost::serialization::make_nvp("calculate_penetration", g.calculate_penetration);
  ar& boost::serialization::make_nvp("calculate_distance", g.calculate_distance);
  ar& boost::serialization::make_nvp("contact_limit", g.contact_limit);
  ar& boost::serialization::make_nvp("is_valid", g.is_valid);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactManagerConfig& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("margin_data_override_type", g.margin_data_override_type);
  ar& boost::serialization::make_nvp("margin_data", g.margin_data);
  ar& boost::serialization::make_nvp("acm", g.acm);
  ar& boost::serialization::make_nvp("acm_override_type", g.acm_override_type);
  ar& boost::serialization::make_nvp("modify_object_enabled", g.modify_object_enabled);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::CollisionCheckConfig& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("contact_manager_config", g.contact_manager_config);
  ar& boost::serialization::make_nvp("contact_request", g.contact_request);
  ar& boost::serialization::make_nvp("type", g.type);
  ar& boost::serialization::make_nvp("longest_valid_segment_length", g.longest_valid_segment_length);
  ar& boost::serialization::make_nvp("check_program_mode", g.check_program_mode);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectorySubstepResults& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("contacts", g.contacts);
  ar& boost::serialization::make_nvp("substep", g.substep);
  ar& boost::serialization::make_nvp("state0", g.state0);
  ar& boost::serialization::make_nvp("state1", g.state1);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectoryStepResults& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("substeps", g.substeps);
  ar& boost::serialization::make_nvp("step", g.step);
  ar& boost::serialization::make_nvp("state0", g.state0);
  ar& boost::serialization::make_nvp("state1", g.state1);
  ar& boost::serialization::make_nvp("total_substeps", g.total_substeps);
}

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactTrajectoryResults& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("steps", g.steps);
  ar& boost::serialization::make_nvp("joint_names", g.joint_names);
  ar& boost::serialization::make_nvp("total_steps", g.total_steps);
}
}  // namespace boost::serialization

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactResult)
TESSERACT_SERIALIZE_SAVE_LOAD_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactResultMap)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactRequest)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactManagerConfig)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::CollisionCheckConfig)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactTrajectorySubstepResults)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactTrajectoryStepResults)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactTrajectoryResults)

TESSERACT_ANY_EXPORT_IMPLEMENT(TesseractCollisionContactResult)
TESSERACT_ANY_EXPORT_IMPLEMENT(TesseractCollisionContactResultMap)
TESSERACT_ANY_EXPORT_IMPLEMENT(TesseractCollisionContactResultMapVector)
