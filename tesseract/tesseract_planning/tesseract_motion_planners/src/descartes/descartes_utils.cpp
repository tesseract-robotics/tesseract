/**
 * @file descartes_utils.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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

#include <tesseract_motion_planners/descartes/descartes_utils.h>

namespace tesseract_planning
{
tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                  double resolution,
                                                  const Eigen::Vector3d& axis)
{
  tesseract_common::VectorIsometry3d samples;
  int cnt = static_cast<int>(std::ceil(2.0 * M_PI / resolution)) + 1;
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(cnt, -M_PI, M_PI);
  samples.reserve(static_cast<size_t>(angles.size()) - 1ul);
  for (long i = 0; i < static_cast<long>(angles.size() - 1); ++i)
  {
    Eigen::Isometry3d p = tool_pose * Eigen::AngleAxisd(angles(i), axis);
    samples.push_back(p);
  }
  return samples;
}

tesseract_common::VectorIsometry3d sampleToolXAxis(const Eigen::Isometry3d& tool_pose, double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitX());
}

tesseract_common::VectorIsometry3d sampleToolYAxis(const Eigen::Isometry3d& tool_pose, double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitY());
}

tesseract_common::VectorIsometry3d sampleToolZAxis(const Eigen::Isometry3d& tool_pose, double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitZ());
}

tesseract_common::VectorIsometry3d sampleFixed(const Eigen::Isometry3d& tool_pose)
{
  return tesseract_common::VectorIsometry3d({ tool_pose });
}
}  // namespace tesseract_planning
