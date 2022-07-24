/**
 * @file srdf_model.h
 * @brief Parse srdf xml
 *
 * @author Levi Armstrong, Ioan Sucan
 * @date May 12, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#ifndef TESSERACT_SRDF_SRDF_MODEL_H
#define TESSERACT_SRDF_SRDF_MODEL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <string>
#include <memory>
#include <array>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/collision_margin_data.h>

/// Main namespace
namespace tesseract_srdf
{
using CollisionMarginData = tesseract_common::CollisionMarginData;
using PairsCollisionMarginData = tesseract_common::PairsCollisionMarginData;

/** @brief Representation of semantic information about the robot */
class SRDFModel
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<SRDFModel>;
  using ConstPtr = std::shared_ptr<const SRDFModel>;

  SRDFModel() = default;
  virtual ~SRDFModel() = default;
  SRDFModel(const SRDFModel&) = default;
  SRDFModel& operator=(const SRDFModel&) = default;
  SRDFModel(SRDFModel&&) = default;
  SRDFModel& operator=(SRDFModel&&) = default;

  /**
   * @brief Load Model given a filename
   * @throws std::nested_exception if an error occurs during parsing srdf
   */
  void initFile(const tesseract_scene_graph::SceneGraph& scene_graph,
                const std::string& filename,
                const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load Model from a XML-string
   * @throws std::nested_exception if an error occurs during parsing srdf
   */
  void initString(const tesseract_scene_graph::SceneGraph& scene_graph,
                  const std::string& xmlstring,
                  const tesseract_common::ResourceLocator& locator);

  /** @brief Save the model to a file */
  bool saveToFile(const std::string& file_path) const;

  /** @brief Clear the model */
  void clear();

  /** @brief The name of the srdf model */
  std::string name{ "undefined" };

  /** @brief The version number major.minor[.patch] */
  std::array<int, 3> version{ { 1, 0, 0 } };

  /** @brief Contact information related to kinematics */
  KinematicsInformation kinematics_information;

  /** @brief The contact managers plugin information */
  tesseract_common::ContactManagersPluginInfo contact_managers_plugin_info;

  /** @brief The allowed collision matrix */
  tesseract_common::AllowedCollisionMatrix acm;

  /** @brief Collision margin data */
  tesseract_common::CollisionMarginData::Ptr collision_margin_data;

  /** @brief The calibration information */
  tesseract_common::CalibrationInfo calibration_info;

  bool operator==(const SRDFModel& rhs) const;
  bool operator!=(const SRDFModel& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_srdf

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_srdf::SRDFModel, "SRDFModel")

#endif  // TESSERACT_SRDF_SRDF_MODEL_H
