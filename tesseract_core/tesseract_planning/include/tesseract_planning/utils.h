#ifndef TESSERACT_PLANNING_UTILS_H
#define TESSERACT_PLANNING_UTILS_H

#include <tesseract_core/basic_types.h>
#include <tesseract_planning/waypoint_definitions.h>
#include <Eigen/Geometry>
#include <memory>
#include <ros/console.h>

namespace tesseract
{
namespace tesseract_planning
{
inline tesseract::VectorIsometry3d interpolate(const Eigen::Isometry3d& start, const Eigen::Isometry3d& stop, int steps)
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

  tesseract::VectorIsometry3d result;
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
      tesseract::VectorIsometry3d eigen_poses = interpolate(w1.cartesian_position_, w2.cartesian_position_, steps);

      std::vector<WaypointPtr> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
      {
        CartesianWaypointPtr new_waypoint = std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
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
}
#endif  // TESSERACT_PLANNING_UTILS_H
