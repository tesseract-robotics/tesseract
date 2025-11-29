/**
 * @file calibration_info.cpp
 * @brief Calibration Information
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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

#include <tesseract_common/calibration_info.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
void CalibrationInfo::insert(const CalibrationInfo& other)
{
  for (const auto& joint : other.joints)
    joints[joint.first] = joint.second;
}

void CalibrationInfo::clear() { joints.clear(); }

bool CalibrationInfo::empty() const { return joints.empty(); }

bool CalibrationInfo::operator==(const CalibrationInfo& rhs) const
{
  auto isometry_equal = [](const Eigen::Isometry3d& iso_1, const Eigen::Isometry3d& iso_2) {
    return iso_1.isApprox(iso_2, 1e-5);
  };

  bool equal = true;
  equal &= tesseract_common::isIdenticalMap<TransformMap, Eigen::Isometry3d>(joints, rhs.joints, isometry_equal);

  return equal;
}
bool CalibrationInfo::operator!=(const CalibrationInfo& rhs) const { return !operator==(rhs); }
}  // namespace tesseract_common
