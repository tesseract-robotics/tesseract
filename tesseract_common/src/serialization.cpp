/**
 * @file serialization.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/array.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>

namespace boost
{
namespace serialization
{
/*****************************/
/****** Eigen::VectorXd ******/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::VectorXd& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar& BOOST_SERIALIZATION_NVP(rows);
  ar& boost::serialization::make_nvp("data", boost::serialization::make_array(g.data(), rows));
}

template <class Archive>
void load(Archive& ar, Eigen::VectorXd& g, const unsigned int /*version*/)
{
  long rows;
  ar& BOOST_SERIALIZATION_NVP(rows);
  g.resize(rows);
  ar& boost::serialization::make_nvp("data", boost::serialization::make_array(g.data(), rows));
}

template <class Archive>
void serialize(Archive& ar, Eigen::VectorXd& g, const unsigned int version)
{
  split_free(ar, g, version);
}

/*******************************/
/****** Eigen::Isometry3d ******/
/*******************************/

template <class Archive>
void save(Archive& ar, const Eigen::Isometry3d& g, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("xyz", boost::serialization::make_array(g.translation().data(), 3));
  Eigen::Quaterniond q(g.linear());
  ar& boost::serialization::make_nvp("xyzw", boost::serialization::make_array(q.vec().data(), 4));
}

template <class Archive>
void load(Archive& ar, Eigen::Isometry3d& g, const unsigned int /*version*/)
{
  g.setIdentity();
  ar& boost::serialization::make_nvp("xyz", boost::serialization::make_array(g.translation().data(), 3));
  Eigen::Quaterniond q;
  ar& boost::serialization::make_nvp("xyzw", boost::serialization::make_array(q.vec().data(), 4));
  q.normalize();
  g.linear() = q.toRotationMatrix();
}

template <class Archive>
void serialize(Archive& ar, Eigen::Isometry3d& g, const unsigned int version)
{
  split_free(ar, g, version);
}

/*****************************/
/****** Eigen::MatrixX2d *****/
/*****************************/
template <class Archive>
void save(Archive& ar, const Eigen::MatrixX2d& g, const unsigned int /*version*/)
{
  long rows = g.rows();
  ar& BOOST_SERIALIZATION_NVP(rows);
  ar& boost::serialization::make_nvp("data", boost::serialization::make_array(g.data(), rows * 2));
}

template <class Archive>
void load(Archive& ar, Eigen::MatrixX2d& g, const unsigned int /*version*/)
{
  long rows;
  ar& BOOST_SERIALIZATION_NVP(rows);
  g.resize(rows, 2);
  ar& boost::serialization::make_nvp("data", boost::serialization::make_array(g.data(), rows * 2));
}

template <class Archive>
void serialize(Archive& ar, Eigen::MatrixX2d& g, const unsigned int version)
{
  split_free(ar, g, version);
}

}  // namespace serialization
}  // namespace boost

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

template void boost::serialization::save(boost::archive::xml_oarchive&,
                                         const Eigen::VectorXd& g,
                                         const unsigned int version);
template void boost::serialization::load(boost::archive::xml_iarchive& ar,
                                         Eigen::VectorXd& g,
                                         const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_oarchive& ar,
                                              Eigen::VectorXd& g,
                                              const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_iarchive& ar,
                                              Eigen::VectorXd& g,
                                              const unsigned int version);

template void boost::serialization::save(boost::archive::xml_oarchive&,
                                         const Eigen::Isometry3d& g,
                                         const unsigned int version);
template void boost::serialization::load(boost::archive::xml_iarchive& ar,
                                         Eigen::Isometry3d& g,
                                         const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_oarchive& ar,
                                              Eigen::Isometry3d& g,
                                              const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_iarchive& ar,
                                              Eigen::Isometry3d& g,
                                              const unsigned int version);

template void boost::serialization::save(boost::archive::xml_oarchive&,
                                         const Eigen::MatrixX2d& g,
                                         const unsigned int version);
template void boost::serialization::load(boost::archive::xml_iarchive& ar,
                                         Eigen::MatrixX2d& g,
                                         const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_oarchive& ar,
                                              Eigen::MatrixX2d& g,
                                              const unsigned int version);
template void boost::serialization::serialize(boost::archive::xml_iarchive& ar,
                                              Eigen::MatrixX2d& g,
                                              const unsigned int version);
