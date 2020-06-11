#ifndef TESSERACT_PLANNING_DESCARTES_UTILS_H
#define TESSERACT_PLANNING_DESCARTES_UTILS_H

#include <tesseract_common/types.h>
#include <tesseract_command_language/cartesian_waypoint.h>

namespace tesseract_planning
{
using PoseSamplerFn = std::function<tesseract_common::VectorIsometry3d(const Eigen::Isometry3d& tool_pose)>;

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the provided axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @param axis The axis to sample around
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                         const double resolution,
                                                         const Eigen::Vector3d& axis)
{
  tesseract_common::VectorIsometry3d samples;
  int cnt = static_cast<int>(std::ceil(2.0f * M_PI / resolution)) + 1;
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(cnt, -M_PI, M_PI);
  samples.reserve(static_cast<size_t>(angles.size()) - 1ul);
  for (long i = 0; i < static_cast<long>(angles.size() - 1); ++i)
  {
    Eigen::Isometry3d p = tool_pose * Eigen::AngleAxisd(angles(i), axis);
    samples.push_back(p);
  }
  return samples;
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the x axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolXAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitX());
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the y axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolYAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitY());
}

/**
 * @brief Given a tool pose create samples from [-PI, PI) around the z axis.
 * @param tool_pose Tool pose to be sampled
 * @param resolution The resolution to sample at
 * @return A vector of tool poses
 */
inline tesseract_common::VectorIsometry3d sampleToolZAxis(const Eigen::Isometry3d& tool_pose, const double resolution)
{
  return sampleToolAxis(tool_pose, resolution, Eigen::Vector3d::UnitZ());
}

/**
 * @brief Given a waypoint it will return the correct tool pose sampler
 * @param wp The waypoint
 * @return A tool pose sampler function
 */
template <typename FloatType>
inline tesseract_motion_planners::PoseSamplerFn getPoseSampler(const ComponentInfo& component)
{
  tesseract_motion_planners::PoseSamplerFn tool_pose_sampler = nullptr;
  switch (component.getType())
  {
    case static_cast<int>(ComponentTypes::FIXED):
    {
      const auto* local = component.cast_const<FixedComponentInfo>();
      tool_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
        return tesseract_common::VectorIsometry3d({ tool_pose });
      };
      break;
    }
    case static_cast<int>(ComponentTypes::CARTESIAN_X_ROTATION_FREE):
    {
      const auto* local = component.cast_const<CartesianXRotationFreeComponentInfo>();
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolXAxis, std::placeholders::_1, local->resolution);
      break;
    }
    case static_cast<int>(ComponentTypes::CARTESIAN_Y_ROTATION_FREE):
    {
      const auto* local = component.cast_const<CartesianYRotationFreeComponentInfo>();
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolYAxis, std::placeholders::_1, local->resolution);
      break;
    }
    case static_cast<int>(ComponentTypes::CARTESIAN_Z_ROTATION_FREE):
    {
      const auto* local = component.cast_const<CartesianXRotationFreeComponentInfo>();
      tool_pose_sampler =
          std::bind(&tesseract_motion_planners::sampleToolZAxis, std::placeholders::_1, local->resolution);
      break;
    }
    default:
    {
      return tool_pose_sampler;
    }
  }

  return tool_pose_sampler;
}
}
#endif // TESSERACT_PLANNING_DESCARTES_UTILS_H
