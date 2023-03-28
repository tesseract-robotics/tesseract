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
#ifndef TESSERACT_COLLISION_SERIALIZATION_H
#define TESSERACT_COLLISION_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <variant>
#include <fstream>
#include <sstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

namespace boost::serialization
{
/************************************************/
/****** tesseract_collision::ContactResult ******/
/************************************************/

template <class Archive>
void save(Archive& ar, const tesseract_collision::ContactResult& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, tesseract_collision::ContactResult& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactResult& g, const unsigned int version);  // NOLINT

template <class Archive>
void save(Archive& ar, const tesseract_collision::ContactResultMap& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, tesseract_collision::ContactResultMap& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, tesseract_collision::ContactResultMap& g, const unsigned int version);  // NOLINT
}  // namespace boost::serialization

#endif  // TESSERACT_COLLISION_SERIALIZATION_H
