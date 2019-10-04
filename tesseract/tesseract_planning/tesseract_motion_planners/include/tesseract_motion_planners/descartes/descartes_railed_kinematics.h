/**
 * @file descartes_railed_kinematics.h
 * @brief Tesseract Descartes Railed Kinematics
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_RAILED_KINEMATICS_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_RAILED_KINEMATICS_SAMPLER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/utils.h>
#include <Eigen/Dense>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
/**
 * @brief This takes a kinematics object that represents the railed system and the robot and adds rail sampling
 *
 * This class is intended to be used for a robots mounted on railed system.
 */
template <typename FloatType>
class DescartesRailedKinematics : public descartes_light::KinematicsInterface<FloatType>
{
public:
  /**
   * @brief This takes a kinematics object that represents the railed system and the robot and adds gantry sampling
   *
   * @param railed_kinematics The kinematic object that the robot is attached to.
   * @param robot_kinematics The kinematic object attached to an railed kinematics object that gets sampled
   * @param world_to_railed_base The transformation from the world coordinate system to the origin of the railed
   * kinematics object
   * @param rail_sample_resolution The resolution at which to sample the railed system at
   * @param robot_reach This defines how far the robot can reach.
   */
  DescartesRailedKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr railed_kinematics,
                            const typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
                            const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_railed_base,
                            const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& railed_sample_resolution,
                            const FloatType robot_reach);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;

  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  bool ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
            const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& railed_pose,
            std::vector<FloatType>& solution_set) const;

  bool fkAt(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& railed_pose,
            const std::vector<FloatType>& pose,
            Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics_;
  tesseract_kinematics::ForwardKinematics::ConstPtr railed_kinematics_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_railed_base_;
  Eigen::Matrix2d railed_limits_;
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> railed_sample_resolution_;
  FloatType robot_reach_;

  void nested_ik(const int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                 Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
                 std::vector<FloatType>& solution_set) const;
};

using DescartesRailedKinematicsD = DescartesRailedKinematics<double>;
using DescartesRailedKinematicsF = DescartesRailedKinematics<float>;
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_RAILED_KINEMATICS_SAMPLER_H
