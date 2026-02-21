/**
 * @file forward_kinematics.h
 * @brief Forward kinematics functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
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

#include <tesseract_kinematics/core/forward_kinematics.h>

#include <tesseract_common/eigen_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tesseract::kinematics
{
tesseract::common::TransformMap
ForwardKinematics::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  tesseract::common::TransformMap transforms;
  calcFwdKin(transforms, joint_angles);
  return transforms;
}  // LCOV_EXCL_LINE

Eigen::MatrixXd ForwardKinematics::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                                const std::string& link_name) const
{
  Eigen::MatrixXd jacobian(6, numJoints());
  calcJacobian(jacobian, joint_angles, link_name);
  return jacobian;
}
}  // namespace tesseract::kinematics
