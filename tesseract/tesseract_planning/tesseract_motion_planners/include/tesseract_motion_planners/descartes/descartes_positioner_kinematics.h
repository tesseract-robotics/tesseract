/**
 * @file descartes_positioner_kinematics.h
 * @brief Tesseract Descartes Robot + External Positioner Kinematics
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_KINEMATICS_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_KINEMATICS_H

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
 * @brief This takes a kinematics object that represents the positioner system and the robot and adds positioner sampling
 *
 * This class is intended to be used for a robots + external positioner.
 */
template <typename FloatType>
class DescartesPositionerKinematics : public descartes_light::KinematicsInterface<FloatType>
{
public:
  /**
   * @brief This takes a kinematics object that represents the positioner system and the robot and adds positioner sampling
   *
   * @param positioner_kinematics The kinematic object of the positioner that is indepened of the robot.
   * @param robot_kinematics The kinematic object of the robot
   * @param world_to_positioner_base The transformation from the world coordinate system to the origin of the positioner
   * kinematics object
   * @param world_to_robot_base The transformation from the world coordinate system to the origin of the robot
   * kinematics object
   * @param positioner_sample_resolution The resolution at which to sample the positioner system at
   * @param point_link_name The name of the link that poses will be relative to. This link must be attached to the positioner
   * @param robot_reach This defines how far the robot can reach.
   * @param positioner_point_link_name_transform The static transform from the positioner tip link to the link frame that the point are relative to
   */
  DescartesPositionerKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
                                const typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
                                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_positioner_base,
                                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_robot_base,
                                const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& positioner_sample_resolution,
                                const std::string point_link_name,
                                const FloatType robot_reach,
                                const Eigen::Transform<FloatType, 3, Eigen::Isometry> positioner_point_link_name_transform = Eigen::Transform<FloatType, 3, Eigen::Isometry>::Identity());

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;

  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  bool ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
            const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
            std::vector<FloatType>& solution_set) const;

  bool fkAt(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
            const std::vector<FloatType>& pose,
            Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics_;
  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> positioner_point_link_name_transform_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_positioner_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base_;
  Eigen::Matrix2d positioner_limits_;
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_sample_resolution_;
  FloatType robot_reach_;
  std::string point_link_name_;

  void nested_ik(const int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                 Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
                 std::vector<FloatType>& solution_set) const;
};

using DescartesPositionerKinematicsD = DescartesPositionerKinematics<double>;
using DescartesPositionerKinematicsF = DescartesPositionerKinematics<float>;
}  // namespace tesseract_motion_planners

#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_KINEMATICS_H
