#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_samplers/samplers/railed_axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/railed_cartesian_point_sampler.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/cartesian_point_sampler.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>
#include <console_bridge/console.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
/**
 * @brief Make a vector of robot position samplers from a vector of waypoints.
 *
 * This chooses a position sampler based on the waypoint type.
 *
 * @param path A vector of waypoints
 * @param kinematic_interface The descartes kinematic interface to use
 * @param collision_interface The descartes collision interface to use.
 * @param radial_sample_resolution The radial sampling resolution. This is required any of the waypoints have an axis
 * that is free to rotate.
 * @return A vector of descartes position samplers.
 */
template <typename FloatType>
std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>
makeRobotPositionSamplers(const std::vector<Waypoint::Ptr>& path,
                          const typename descartes_light::KinematicsInterface<FloatType>::Ptr& kinematic_interface,
                          const typename descartes_light::CollisionInterface<FloatType>::Ptr& collision_interface,
                          const FloatType radial_sample_resolution = 60 * M_PI / 180)
{
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  result.reserve(path.size());
  for (const auto& wp : path)
  {
    if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
    {
      CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);
      if (wp->getCoefficients().size() == 0 || (wp->getCoefficients().array() > 0).all())  // Fixed pose
      {
        if (collision_interface == nullptr)
        {
          auto sampler = std::make_shared<descartes_light::CartesianPointSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, nullptr, true);
          result.push_back(std::move(sampler));
        }
        else
        {
          auto sampler = std::make_shared<descartes_light::CartesianPointSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, collision_interface->clone(), true);
          result.push_back(std::move(sampler));
        }
      }
      else if ((wp->getCoefficients().head(5).array() > 0).all() && !(wp->getCoefficients()(5) > 0))
      {
        if (collision_interface == nullptr)
        {
          auto sampler = std::make_shared<descartes_light::AxialSymmetricSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, radial_sample_resolution, nullptr, true);
          result.push_back(std::move(sampler));
        }
        else
        {
          auto sampler =
              std::make_shared<descartes_light::AxialSymmetricSampler<FloatType>>(cwp->getTransform().cast<FloatType>(),
                                                                                  kinematic_interface,
                                                                                  radial_sample_resolution,
                                                                                  collision_interface->clone(),
                                                                                  true);
          result.push_back(std::move(sampler));
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not support the provided under constrained cartesian "
                                "pose!");
        return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
      }
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      std::vector<FloatType> joint_pose(jwp->getPositions().data(),
                                        jwp->getPositions().data() +
                                            jwp->getPositions().rows() * jwp->getPositions().cols());
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
 * @brief Make a vector of gantry position samplers from a vector of waypoints.
 *
 * This chooses a position sampler based on the waypoint type. Also the kinematics interface should only be for the
 * robot when using this utility function.
 *
 * @param path A vector of waypoints
 * @param kinematic_interface The descartes kinematic interface to use
 * @param collision_interface The descartes collision interface to use.
 * @param radial_sample_resolution The radial sampling resolution. This is required any of the waypoints have an axis
 * that is free to rotate.
 * @return A vector of descartes position samplers.
 */
template <typename FloatType>
std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>
makeGantryPositionSamplers(const std::vector<Waypoint::Ptr>& path,
                           const typename descartes_light::KinematicsInterface<FloatType>::Ptr& kinematic_interface,
                           const typename descartes_light::CollisionInterface<FloatType>::Ptr& collision_interface,
                           const FloatType radial_sample_resolution = 60 * M_PI / 180)
{
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
  result.reserve(path.size());
  for (const auto& wp : path)
  {
    if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
    {
      CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);
      if (wp->getCoefficients().size() == 0 || (wp->getCoefficients().array() > 0).all())  // Fixed pose
      {
        if (collision_interface == nullptr)
        {
          auto sampler = std::make_shared<descartes_light::RailedCartesianPointSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, nullptr, true);
          result.push_back(std::move(sampler));
        }
        else
        {
          auto sampler = std::make_shared<descartes_light::RailedCartesianPointSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, collision_interface->clone(), true);
          result.push_back(std::move(sampler));
        }
      }
      else if ((wp->getCoefficients().head(5).array() > 0).all() && !(wp->getCoefficients()(5) > 0))
      {
        if (collision_interface == nullptr)
        {
          auto sampler = std::make_shared<descartes_light::RailedAxialSymmetricSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(), kinematic_interface, radial_sample_resolution, nullptr, true);
          result.push_back(std::move(sampler));
        }
        else
        {
          auto sampler = std::make_shared<descartes_light::RailedAxialSymmetricSampler<FloatType>>(
              cwp->getTransform().cast<FloatType>(),
              kinematic_interface,
              radial_sample_resolution,
              collision_interface->clone(),
              true);
          result.push_back(std::move(sampler));
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not support the provided under constrained cartesian "
                                "pose!");
        return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
      }
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      std::vector<FloatType> joint_pose(jwp->getPositions().data(),
                                        jwp->getPositions().data() +
                                            jwp->getPositions().rows() * jwp->getPositions().cols());
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
