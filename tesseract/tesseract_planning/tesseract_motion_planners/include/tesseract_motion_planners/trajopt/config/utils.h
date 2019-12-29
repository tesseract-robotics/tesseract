/**
 * @file utils.h
 * @brief Utilities for creating TrajOpt term information
 *
 * @author Michael Ripperger
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_UTILS_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_UTILS_H

#include <trajopt/problem_description.hpp>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
trajopt::TermInfo::Ptr createJointWaypointTermInfo(const JointWaypoint::ConstPtr& waypoint,
                                                   int ind,
                                                   const std::vector<std::string>& joint_names,
                                                   double coeff = 1.0,
                                                   const std::string& name = "joint_position");

trajopt::TermInfo::Ptr createJointTolerancedWaypointTermInfo(const JointTolerancedWaypoint::ConstPtr& waypoint,
                                                             int ind,
                                                             const std::vector<std::string>& joint_names,
                                                             double coeff = 0.1,
                                                             const std::string& name = "joint_toleranced_position");

trajopt::TermInfo::Ptr createCartesianWaypointTermInfo(const CartesianWaypoint::ConstPtr& waypoint,
                                                       int ind,
                                                       const std::string& link,
                                                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
                                                       const std::string& name = "cartesian_position_position");

trajopt::TermInfo::Ptr
createDynamicCartesianWaypointTermInfo(const CartesianWaypoint::ConstPtr& waypoint,
                                       int ind,
                                       const std::string& link,
                                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
                                       const std::string& name = "dynamic_cartesian_position");

struct WaypointTermInfo
{
  using TermInfoVec = typename std::vector<trajopt::TermInfo::Ptr>;

  TermInfoVec cnt;
  TermInfoVec cost;
};

WaypointTermInfo createWaypointTermInfo(const Waypoint::ConstPtr& waypoint,
                                        int ind,
                                        const std::vector<std::string>& joint_names,
                                        const std::vector<std::string>& adjacency_map_links,
                                        const std::string& link,
                                        const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity());

trajopt::TermInfo::Ptr createConfigurationTermInfo(const JointWaypoint::ConstPtr& configuration,
                                                   const std::vector<std::string>& joint_names,
                                                   int n_steps,
                                                   double coeff = 1.0,
                                                   const std::string& name = "configuration_cost");

trajopt::TermInfo::Ptr createCollisionTermInfo(
    int n_steps,
    double collision_safety_margin,
    bool collision_continuous = true,
    double coeff = 20.0,
    tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL,
    double longest_valid_segment_length = 0.5,
    const std::string& name = "collision_cost");

trajopt::TermInfo::Ptr
createSmoothVelocityTermInfo(int n_steps, int n_joints, double coeff = 5.0, const std::string& name = "joint_vel_cost");

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int n_steps,
                                                    const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                    const std::string& name = "joint_vel_cost");

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int n_steps,
                                                        int n_joints,
                                                        double coeff = 1.0,
                                                        const std::string& name = "joint_accel_cost");

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int n_steps,
                                                        const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                        const std::string& name = "joint_accel_cost");

trajopt::TermInfo::Ptr
createSmoothJerkTermInfo(int n_steps, int n_joints, double coeff = 1.0, const std::string& name = "joint_jerk_cost");

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int n_steps,
                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                const std::string& name = "joint_jerk_cost");

trajopt::TermInfo::Ptr createUserDefinedTermInfo(int n_steps,
                                                 sco::VectorOfVector::func error_function,
                                                 sco::MatrixOfVector::func jacobian_function,
                                                 const std::string& name = "user_defined");

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_UTILS_H
