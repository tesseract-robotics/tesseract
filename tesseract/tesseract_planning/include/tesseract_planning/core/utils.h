/**
 * @file utils.h
 * @brief Planner utility functions.
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
#ifndef TESSERACT_PLANNING_UTILS_H
#define TESSERACT_PLANNING_UTILS_H

#include <tesseract_planning/core/macros.h>
TESSERACT_PLANNING_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_PLANNING_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_planning/core/waypoint.h>

namespace tesseract_planning
{
inline tesseract_environment::VectorIsometry3d interpolate(const Eigen::Isometry3d& start, const Eigen::Isometry3d& stop, int steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  tesseract_environment::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps + 1));
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

inline std::vector<WaypointPtr> interpolate(const Waypoint& start, const Waypoint& stop, int steps)
{
  switch (start.getType())
  {
    case WaypointType::CARTESIAN_WAYPOINT:
    {
      const CartesianWaypoint& w1 = static_cast<const CartesianWaypoint&>(start);
      const CartesianWaypoint& w2 = static_cast<const CartesianWaypoint&>(stop);
      tesseract_environment::VectorIsometry3d eigen_poses = interpolate(w1.cartesian_position_, w2.cartesian_position_, steps);

      std::vector<WaypointPtr> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
      {
        CartesianWaypointPtr new_waypoint = std::make_shared<tesseract_planning::CartesianWaypoint>();
        new_waypoint->cartesian_position_ = eigen_pose;
        new_waypoint->coeffs_ = start.coeffs_;
        new_waypoint->is_critical_ = start.is_critical_;
        result.push_back(new_waypoint);
      }

      return result;
    }
    default:
    {
      ROS_ERROR("Interpolator for Waypoint type %d is currently not support!", start.getType());
      return std::vector<WaypointPtr>();
    }
  }
}
}

#endif  // TESSERACT_PLANNING_UTILS_H
