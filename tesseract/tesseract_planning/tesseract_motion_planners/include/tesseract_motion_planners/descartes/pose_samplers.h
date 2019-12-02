/**
 * @file pose_samplers.h
 * @brief Tesseract pose sampers
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_POSE_SAMPLERS_H
#define TESSERACT_MOTION_PLANNERS_POSE_SAMPLERS_H

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <type_traits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_motion_planners
{
using PoseSamplerFn = std::function<tesseract_common::VectorIsometry3d(const Eigen::Isometry3d& tool_pose)>;

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the provided axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param axis The axis to sample around
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                         const double resolution,
                                                         const Eigen::Vector3d& axis)
{
  tesseract_common::VectorIsometry3d samples;
  int cnt = static_cast<int>(std::ceil(2.0f * M_PI / resolution)) + 1;
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(cnt, -M_PI, M_PI);
  samples.reserve(static_cast<size_t>(angles.size()) - 1ul);
  for (long i = 0; i < static_cast<long>(angles.size() - 1); ++i)
  {
    Eigen::Isometry3d p = tool_pose * Eigen::AngleAxisd(angles(i), axis);
    samples.push_back(p);
  }
  return samples;
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the x axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolXAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitX());
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the y axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolYAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitY());
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the z axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolZAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitZ());
}

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_POSE_SAMPLERS_H
