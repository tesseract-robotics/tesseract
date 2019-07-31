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

template<typename FloatType>
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
      if (wp->getCoefficients().size() == 0 || (wp->getCoefficients().array() > 0).all()) // Fixed pose
      {
        auto sampler = std::make_shared<descartes_light::CartesianPointSampler<FloatType>>(cwp->getTransform().cast<FloatType>(), kinematic_interface, collision_interface->clone(), true);
        result.push_back(std::move(sampler));
      }
      else if ((wp->getCoefficients().head(5).array() > 0).all() && !(wp->getCoefficients()(5) > 0))
      {
        auto sampler = std::make_shared<descartes_light::AxialSymmetricSampler<FloatType>>(cwp->getTransform().cast<FloatType>(), kinematic_interface, radial_sample_resolution, collision_interface->clone(), true);
        result.push_back(std::move(sampler));
      }
      else
      {
        CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not support the provided under constrained cartesian pose!");
        return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
      }
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      std::vector<FloatType> joint_pose(jwp->getPositions().data(), jwp->getPositions().data() + jwp->getPositions().rows() * jwp->getPositions().cols());
      auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
      result.push_back(std::move(sampler));
    }
    else
    {
      CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not currently support waypoint type: %d", wp->getType());
      return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
    }
  }

  return result;
}

template<typename FloatType>
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
      if (wp->getCoefficients().size() == 0 || (wp->getCoefficients().array() > 0).all()) // Fixed pose
      {
        auto sampler = std::make_shared<descartes_light::RailedCartesianPointSampler<FloatType>>(cwp->getTransform().cast<FloatType>(), kinematic_interface, collision_interface->clone(), true);
        result.push_back(std::move(sampler));
      }
      else if ((wp->getCoefficients().head(5).array() > 0).all() && !(wp->getCoefficients()(5) > 0))
      {
        auto sampler = std::make_shared<descartes_light::RailedAxialSymmetricSampler<FloatType>>(cwp->getTransform().cast<FloatType>(), kinematic_interface, radial_sample_resolution, collision_interface->clone(), true);
        result.push_back(std::move(sampler));
      }
      else
      {
        CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not support the provided under constrained cartesian pose!");
        return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
      }
    }
    else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
    {
      JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
      std::vector<FloatType> joint_pose(jwp->getPositions().data(), jwp->getPositions().data() + jwp->getPositions().rows() * jwp->getPositions().cols());
      auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
      result.push_back(std::move(sampler));
    }
    else
    {
      CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not currently support waypoint type: %d", wp->getType());
      return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
    }
  }

  return result;
}
}

#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_UTILS_H
