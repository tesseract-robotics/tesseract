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
#ifndef TESSERACT_PLANNING_DESCARTES_UTILS_H
#define TESSERACT_PLANNING_DESCARTES_UTILS_H

#include <tesseract_common/types.h>
#include <tesseract_command_language/cartesian_waypoint.h>

namespace tesseract_planning
{
using PoseSamplerFn = std::function<tesseract_common::VectorIsometry3d(const Eigen::Isometry3d& tool_pose)>;

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the provided axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param axis The axis to sample around
 * @return A vector of tool poses
 */
tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                  double resolution,
                                                  const Eigen::Vector3d& axis);

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the x axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
tesseract_common::VectorIsometry3d sampleToolXAxis(const Eigen::Isometry3d& tool_pose, double resolution);

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the y axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
tesseract_common::VectorIsometry3d sampleToolYAxis(const Eigen::Isometry3d& tool_pose, double resolution);

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the z axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
tesseract_common::VectorIsometry3d sampleToolZAxis(const Eigen::Isometry3d& tool_pose, double resolution);

/**
 * @brief This is the default sample with if a fixed pose sampler
 * @param tool_pose Tool pose to be sampled
 * @return A vector with a single pose that was provided as input to function
 */
tesseract_common::VectorIsometry3d sampleFixed(const Eigen::Isometry3d& tool_pose);

}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_DESCARTES_UTILS_H
