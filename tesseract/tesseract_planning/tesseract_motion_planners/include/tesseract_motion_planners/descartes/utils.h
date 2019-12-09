/**
 * @file utils.h
 * @brief Tesseract descartes utility functions
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>
#include <descartes_light/ladder_graph.h>
#include <console_bridge/console.h>
#include <vector>
#include <Eigen/Geometry>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_robot_positioner_sampler.h>
#include <tesseract_motion_planners/descartes/types.h>

namespace tesseract_motion_planners
{
/**
 * @brief Given a waypoint it will return the correct tool pose sampler
 * @param wp The waypoint
 * @param x_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample x-axis
 * at
 * @param y_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample y-axis
 * at
 * @param z_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample z-axis
 * at
 * @param x_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * x-axis rotation at
 * @param y_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * y-axis rotation at
 * @param z_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * z-axis rotation at
 * @return A tool pose sampler function
 */
template <typename FloatType>
inline tesseract_motion_planners::PoseSamplerFn
getPoseSampler(const Waypoint::ConstPtr& wp,
               const FloatType x_sample_resolution = 0.001,
               const FloatType y_sample_resolution = 0.001,
               const FloatType z_sample_resolution = 0.001,
               const FloatType x_axes_sample_resolution = 60 * M_PI / 180,
               const FloatType y_axes_sample_resolution = 60 * M_PI / 180,
               const FloatType z_axes_sample_resolution = 60 * M_PI / 180)
{
  UNUSED(x_sample_resolution);
  UNUSED(y_sample_resolution);
  UNUSED(z_sample_resolution);
  tesseract_motion_planners::PoseSamplerFn tool_pose_sampler = nullptr;
  if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
  {
    CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);
    if (wp->getCoefficients().size() == 0 || (wp->getCoefficients().array() > 0).all())  // Fixed pose
    {
      tool_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
        return tesseract_common::VectorIsometry3d({ tool_pose });
      };
    }
    else if ((wp->getCoefficients().head(3).array() > 0).all() && !(wp->getCoefficients()(3) > 0) &&
             (wp->getCoefficients()(4) > 0) && (wp->getCoefficients()(5) > 0))
    {
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolXAxis, std::placeholders::_1, x_axes_sample_resolution);
    }
    else if ((wp->getCoefficients().head(3).array() > 0).all() && (wp->getCoefficients()(3) > 0) &&
             !(wp->getCoefficients()(4) > 0) && (wp->getCoefficients()(5) > 0))
    {
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolYAxis, std::placeholders::_1, y_axes_sample_resolution);
    }
    else if ((wp->getCoefficients().head(3).array() > 0).all() && (wp->getCoefficients()(3) > 0) &&
             (wp->getCoefficients()(4) > 0) && !(wp->getCoefficients()(5) > 0))
    {
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolZAxis, std::placeholders::_1, z_axes_sample_resolution);
    }
    else
    {
      CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not support the provided under constrained cartesian "
                              "pose!");
    }
  }

  return tool_pose_sampler;
}

/**
 * @brief Make a vector of position samplers from a vector of waypoints for a robot
 * @param path A vector of waypoints
 * @param robot_kinematics The robot inverse kinematics
 * @param collision_interface The descartes collision interface to use
 * @param current_state The current state of the environment
 * @param robot_tcp The robots tcp
 * @param robot_reach The reach of the robot. This is used by descartes to filter samples base on distace
 * @param allow_collision Enable descartes option to return a solution in collision if one does not exist
 * @param is_valid This is the is valid function used by descartes to check if a solution should be saved
 * @param x_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample x-axis
 * at
 * @param y_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample y-axis
 * at
 * @param z_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample z-axis
 * at
 * @param x_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * x-axis rotation at
 * @param y_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * y-axis rotation at
 * @param z_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * z-axis rotation at
 * @return
 */
template <typename FloatType>
std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>
makeRobotSamplers(const std::vector<Waypoint::Ptr>& path,
                  const tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
                  const typename descartes_light::CollisionInterface<FloatType>::Ptr& collision_interface,
                  const tesseract_environment::EnvState::ConstPtr current_state,
                  const Eigen::Isometry3d robot_tcp,
                  const double robot_reach,
                  const bool allow_collision,
                  const DescartesIsValidFn<FloatType>& is_valid,
                  const FloatType x_sample_resolution = 0.001,
                  const FloatType y_sample_resolution = 0.001,
                  const FloatType z_sample_resolution = 0.001,
                  const FloatType x_axes_sample_resolution = 60 * M_PI / 180,
                  const FloatType y_axes_sample_resolution = 60 * M_PI / 180,
                  const FloatType z_axes_sample_resolution = 60 * M_PI / 180)
{
  const std::vector<std::string>& joint_names = robot_kinematics->getJointNames();
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  result.reserve(path.size());

  for (const auto& wp : path)
  {
    typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
    if (collision_interface != nullptr)
      ci = collision_interface->clone();

    tesseract_motion_planners::PoseSamplerFn target_pose_sampler = getPoseSampler(wp,
                                                                                  x_sample_resolution,
                                                                                  y_sample_resolution,
                                                                                  z_sample_resolution,
                                                                                  x_axes_sample_resolution,
                                                                                  y_axes_sample_resolution,
                                                                                  z_axes_sample_resolution);
    if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
    {
      CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);

      // Check if the waypoint is not relative to the world coordinate system
      Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
      if (!cwp->getParentLinkName().empty())
        world_to_waypoint = current_state->transforms.at(cwp->getParentLinkName());

      auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(world_to_waypoint * cwp->getTransform(),
                                                                        target_pose_sampler,
                                                                        robot_kinematics,
                                                                        ci,
                                                                        current_state,
                                                                        robot_tcp,
                                                                        robot_reach,
                                                                        allow_collision,
                                                                        is_valid);
      result.push_back(std::move(sampler));
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      Eigen::Matrix<FloatType, 1, Eigen::Dynamic> jwp_positions = jwp->getPositions(joint_names).cast<FloatType>();
      std::vector<FloatType> joint_pose(jwp_positions.data(),
                                        jwp_positions.data() + jwp_positions.rows() * jwp_positions.cols());
      auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
      result.push_back(std::move(sampler));
    }
    else
    {
      CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not currently support waypoint type: %d",
                              wp->getType());
      return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
    }
  }

  return result;
}

/**
 * @brief Make a vector of position samplers from a vector of waypoints for a robot attached to a kinematic positioner
 *
 *  This is to be used if you have a robot attached to a rail, gantry, revolute positioner, mobile base, ext. where the
 *  forward kinematics object represent the kinematics of the object the robot is attached to. The tip of the forward
 *  kinematics object must match the base link of the inverse kinematics object for the robot.
 *
 * @param path A vector of waypoints
 * @param railed_kinematics The robot base positioner (rail, gantry, revolute positioner, etc) kinematics
 * @param robot_kinematics The robot inverse kinematics
 * @param collision_interface The descartes collision interface to use
 * @param current_state The current state of the environment
 * @param railed_sample_resolution The resolution to sample the base positioner kinematics.
 * @param robot_tcp The robots tcp
 * @param robot_reach The reach of the robot. This is used by descartes to filter samples base on distace
 * @param allow_collision Enable descartes option to return a solution in collision if one does not exist
 * @param is_valid This is the is valid function used by descartes to check if a solution should be saved
 * @param x_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample x-axis
 * at
 * @param y_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample y-axis
 * at
 * @param z_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample z-axis
 * at
 * @param x_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * x-axis rotation at
 * @param y_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * y-axis rotation at
 * @param z_axes_sample_resolution If a toleranced cartesian waypoint is provided this is the max resolution to sample
 * z-axis rotation at
 * @return
 */
template <typename FloatType>
std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>
makeRobotPositionerSamplers(const std::vector<Waypoint::Ptr>& path,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
                            const tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
                            const typename descartes_light::CollisionInterface<FloatType>::Ptr& collision_interface,
                            const tesseract_environment::EnvState::ConstPtr current_state,
                            const Eigen::VectorXd positioner_sample_resolution,
                            const Eigen::Isometry3d robot_tcp,
                            const double robot_reach,
                            const bool allow_collision,
                            const DescartesIsValidFn<FloatType>& is_valid,
                            const FloatType x_sample_resolution = 0.001,
                            const FloatType y_sample_resolution = 0.001,
                            const FloatType z_sample_resolution = 0.001,
                            const FloatType x_axes_sample_resolution = 60 * M_PI / 180,
                            const FloatType y_axes_sample_resolution = 60 * M_PI / 180,
                            const FloatType z_axes_sample_resolution = 60 * M_PI / 180)
{
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  result.reserve(path.size());

  std::vector<std::string> joint_names;
  const std::vector<std::string> gantry_joint_names = positioner_kinematics->getJointNames();
  const std::vector<std::string> robot_joint_names = robot_kinematics->getJointNames();
  joint_names.insert(joint_names.end(), gantry_joint_names.begin(), gantry_joint_names.end());
  joint_names.insert(joint_names.end(), robot_joint_names.begin(), robot_joint_names.end());

  for (const auto& wp : path)
  {
    typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
    if (collision_interface != nullptr)
      ci = collision_interface->clone();

    // Based on the waypoint type pick the tool pose sampler
    tesseract_motion_planners::PoseSamplerFn target_pose_sampler = getPoseSampler(wp,
                                                                                  x_sample_resolution,
                                                                                  y_sample_resolution,
                                                                                  z_sample_resolution,
                                                                                  x_axes_sample_resolution,
                                                                                  y_axes_sample_resolution,
                                                                                  z_axes_sample_resolution);
    if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
    {
      CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);

      // Check if the waypoint is not relative to the world coordinate system
      Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
      if (!cwp->getParentLinkName().empty())
        world_to_waypoint = current_state->transforms.at(cwp->getParentLinkName());

      auto sampler =
          std::make_shared<DescartesRobotPositionerSampler<FloatType>>(world_to_waypoint * cwp->getTransform(),
                                                                       target_pose_sampler,
                                                                       positioner_kinematics,
                                                                       robot_kinematics,
                                                                       ci,
                                                                       current_state,
                                                                       positioner_sample_resolution,
                                                                       robot_tcp,
                                                                       robot_reach,
                                                                       allow_collision,
                                                                       is_valid);
      result.push_back(std::move(sampler));
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      Eigen::Matrix<FloatType, 1, Eigen::Dynamic> jwp_positions = jwp->getPositions(joint_names).cast<FloatType>();
      std::vector<FloatType> joint_pose(jwp_positions.data(),
                                        jwp_positions.data() + jwp_positions.rows() * jwp_positions.cols());
      auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
      result.push_back(std::move(sampler));
    }
    else
    {
      CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not currently support waypoint type: %d",
                              wp->getType());
      return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
    }
  }

  return result;
}

/**
 * @brief Make a vector of Descartes timing constraints from a vector of waypoints
 * @param path The vector of waypoints
 * @param dt The timing constraint to be used for each waypoint
 * @return A vector of Descartes timing constraints
 */
template <typename FloatType>
std::vector<descartes_core::TimingConstraint<FloatType>> makeTiming(const std::vector<Waypoint::Ptr>& path,
                                                                    const double dt)
{
  std::vector<descartes_core::TimingConstraint<FloatType>> timing(path.size(), dt);
  // TODO(jmeyer): Compute the real time
  // In Descartes land, the timing constraint represents how long the dt is between the previous point and the point
  // associated with this particular constraint. In a trajectory with only one pass the first point is meaningless (?).
  // Here I want to append many passes together so setting the DT to 0.0 is sort of saying: "Hey, take as long
  // as you need to get to here from the last point".

  return timing;
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H
