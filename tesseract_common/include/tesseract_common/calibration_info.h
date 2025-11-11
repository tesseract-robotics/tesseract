/**
 * @file calibration_info.h
 * @brief Calibration Information
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_CALIBRATION_INFO_H
#define TESSERACT_COMMON_CALIBRATION_INFO_H

#include <tesseract_common/fwd.h>
#include <tesseract_common/eigen_types.h>

namespace tesseract_common
{
struct CalibrationInfo;

template <class Archive>
void serialize(Archive& ar, CalibrationInfo& obj);

/** @brief The CalibrationInfo struct */
struct CalibrationInfo
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  CalibrationInfo() = default;

  /**
   *  @brief The joint origin calibration information
   *  @details For each entry in \p joints the environment will apply a ChangeJointOriginCommand replacing the current
   * joint origin with what is stored in the TransformMap
   */
  tesseract_common::TransformMap joints;

  /** @brief Insert the content of an other CalibrationInfo */
  void insert(const CalibrationInfo& other);

  /** @brief Clear the contents */
  void clear();

  /** @brief Check if structure is empty */
  bool empty() const;

  // Yaml Config key
  static inline const std::string CONFIG_KEY{ "calibration" };

  bool operator==(const CalibrationInfo& rhs) const;
  bool operator!=(const CalibrationInfo& rhs) const;

private:
  template <class Archive>
  friend void ::tesseract_common::serialize(Archive& ar, CalibrationInfo& obj);
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_CALIBRATION_INFO_H
