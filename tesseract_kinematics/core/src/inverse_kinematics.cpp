/**
 * @file inverse_kinematics.cpp
 * @brief Inverse kinematics functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

#include <tesseract_common/eigen_types.h>

namespace tesseract_kinematics
{
InverseKinematics::~InverseKinematics() = default;

IKSolutions InverseKinematics::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  IKSolutions solutions;
  calcInvKin(solutions, tip_link_poses, seed);
  return solutions;
}  // LCOV_EXCL_LINE
}  // namespace tesseract_kinematics
