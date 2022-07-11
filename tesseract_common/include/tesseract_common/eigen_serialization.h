/**
 * @file serialization.h
 * @brief Additional Boost serialization wrappers
 * @details Supports the following
 *            - Eigen::VectorXd
 *            - Eigen::Isometry3d
 *            - Eigen::MatrixX2d
 *
 * @author Levi Armstrong
 * @date February 21, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_EIGEN_SERIALIZATION_H
#define TESSERACT_COMMON_EIGEN_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <variant>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace boost::serialization
{
/*****************************/
/****** Eigen::VectorXd ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::VectorXd& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::VectorXd& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::VectorXd& g, const unsigned int version);  // NOLINT

/*****************************/
/****** Eigen::Vector3d ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::Vector3d& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::Vector3d& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::Vector3d& g, const unsigned int version);  // NOLINT

/*****************************/
/****** Eigen::Vector4d ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::Vector4d& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::Vector4d& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::Vector4d& g, const unsigned int version);  // NOLINT

/*****************************/
/****** Eigen::VectorXi ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::VectorXi& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::VectorXi& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::VectorXi& g, const unsigned int version);  // NOLINT

/*****************************/
/****** Eigen::VectorXd ******/
/*****************************/

template <class Archive>
void save(Archive& ar, const Eigen::Isometry3d& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::Isometry3d& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::Isometry3d& g, const unsigned int version);  // NOLINT

/*****************************/
/****** Eigen::MatrixX2d *****/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::MatrixX2d& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, Eigen::MatrixX2d& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, Eigen::MatrixX2d& g, const unsigned int version);  // NOLINT

/*********************************************************/
/****** std::variant<std::string, Eigen::Isometry3d> *****/
/*********************************************************/
template <class Archive>
void save(Archive& ar, const std::variant<std::string, Eigen::Isometry3d>& g, const unsigned int version);  // NOLINT

template <class Archive>
void load(Archive& ar, std::variant<std::string, Eigen::Isometry3d>& g, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, std::variant<std::string, Eigen::Isometry3d>& g, const unsigned int version);  // NOLINT

}  // namespace boost::serialization

// Set the tracking to track_never for all Eigen types.
BOOST_CLASS_TRACKING(Eigen::VectorXd, boost::serialization::track_never);
BOOST_CLASS_TRACKING(Eigen::Vector3d, boost::serialization::track_never);
BOOST_CLASS_TRACKING(Eigen::Vector4d, boost::serialization::track_never);
BOOST_CLASS_TRACKING(Eigen::VectorXi, boost::serialization::track_never);
BOOST_CLASS_TRACKING(Eigen::Isometry3d, boost::serialization::track_never);
BOOST_CLASS_TRACKING(Eigen::MatrixX2d, boost::serialization::track_never);

#endif  // TESSERACT_COMMON_SERIALIZATION_H
