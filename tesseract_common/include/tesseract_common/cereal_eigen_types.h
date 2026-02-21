/**
 * @file serialization.h
 * @brief Additional serialization wrappers
 * @details Supports the following
 *            - Eigen::VectorXd
 *            - Eigen::Isometry3d
 *            - Eigen::MatrixX2d
 *
 * @author Levi Armstrong
 * @date February 21, 2021
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
#ifndef TESSERACT_COMMON_CEREAL_EIGEN_TYPES_H
#define TESSERACT_COMMON_CEREAL_EIGEN_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <cereal/cereal.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/cereal_make_array.h>

namespace cereal
{
/*****************************/
/****** Eigen::VectorXd ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::VectorXd& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar(CEREAL_NVP(rows));
  ar& cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows));
}

template <class Archive>
void load(Archive& ar, Eigen::VectorXd& g, const unsigned int /*version*/)
{
  long rows{ 0 };
  ar(CEREAL_NVP(rows));
  g.resize(rows);
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

/*****************************/
/****** Eigen::Vector3d ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::Vector3d& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar(CEREAL_NVP(rows));
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

template <class Archive>
void load(Archive& ar, Eigen::Vector3d& g, const unsigned int /*version*/)
{
  long rows{ 0 };
  ar(CEREAL_NVP(rows));
  g.resize(rows);
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

/*****************************/
/****** Eigen::Vector4d ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::Vector4d& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar(CEREAL_NVP(rows));
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

template <class Archive>
void load(Archive& ar, Eigen::Vector4d& g, const unsigned int /*version*/)
{
  long rows{ 0 };
  ar(CEREAL_NVP(rows));
  g.resize(rows);
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

/*****************************/
/****** Eigen::VectorXi ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::VectorXi& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar(CEREAL_NVP(rows));
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

template <class Archive>
void load(Archive& ar, Eigen::VectorXi& g, const unsigned int /*version*/)
{
  long rows{ 0 };
  ar(CEREAL_NVP(rows));
  g.resize(rows);
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows)));
}

/*******************************/
/****** Eigen::Isometry3d ******/
/*******************************/

template <class Archive>
void save(Archive& ar, const Eigen::Isometry3d& g, const unsigned int /*version*/)
{
  ar(cereal::make_nvp("xyz", tesseract::common::serialization::make_array(g.translation().data(), 3)));
  Eigen::Quaterniond q(g.linear());
  ar(cereal::make_nvp("xyzw", tesseract::common::serialization::make_array(q.vec().data(), 4)));
}

template <class Archive>
void load(Archive& ar, Eigen::Isometry3d& g, const unsigned int /*version*/)
{
  g.setIdentity();
  ar(cereal::make_nvp("xyz", tesseract::common::serialization::make_array(g.translation().data(), 3)));
  Eigen::Quaterniond q;
  ar(cereal::make_nvp("xyzw", tesseract::common::serialization::make_array(q.vec().data(), 4)));
  q.normalize();
  g.linear() = q.toRotationMatrix();
}

/*****************************/
/****** Eigen::MatrixX2d *****/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::MatrixX2d& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar(CEREAL_NVP(rows));
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows * 2)));
}

template <class Archive>
void load(Archive& ar, Eigen::MatrixX2d& g, const unsigned int /*version*/)
{
  long rows{ 0 };
  ar(CEREAL_NVP(rows));
  g.resize(rows, 2);
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), rows * 2)));
}

/****************************************/
/****** Eigen::Matrix<double, 6, 1> *****/
/****************************************/

template <class Archive>
void serialize(Archive& ar, Eigen::Matrix<double, 6, 1>& g, const unsigned int /*version*/)
{
  ar(cereal::make_nvp("data", tesseract::common::serialization::make_array(g.data(), 6)));
}

}  // namespace cereal

#endif  // TESSERACT_COMMON_CEREAL_EIGEN_TYPES_H
