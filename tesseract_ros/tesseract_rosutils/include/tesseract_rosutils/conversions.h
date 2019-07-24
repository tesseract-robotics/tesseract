#ifndef TESSERACT_ROSUTILS_CONVERSIONS_H
#define TESSERACT_ROSUTILS_CONVERSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>
#include <tesseract_msgs/ProcessPlan.h>
#include <tesseract_msgs/ProcessPlanPath.h>
#include <tesseract_msgs/ProcessPlanSegment.h>
#include <tesseract_msgs/ProcessPlanTransitionPair.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_process_planners/process_definition.h>
#include <tesseract_process_planners/process_planner.h>

namespace tesseract_rosutils
{
/**
 * @brief Convert STD Vector to Eigen Vector
 * @param vector The STD Vector to be converted
 * @return Eigen::VectorXd
 */
inline Eigen::VectorXd toEigen(const std::vector<double>& vector)
{
  return Eigen::VectorXd::Map(vector.data(), static_cast<long>(vector.size()));
}

/**
 * @brief Convert Geometry Pose Message to Eigen
 * @param pose Geometry Pose Message
 * @return Eigen Isometry Matrix
 */
inline Eigen::Isometry3d toEigen(const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  return pose_eigen;
}

/**
 * @brief Converts JointState position to Eigen vector in the order provided by joint_names
 * @param joint_state The JointState
 * @param joint_names The vector joint names used to order the output
 * @return Eigen::VectorXd in the same order as joint_names
 */
inline Eigen::VectorXd toEigen(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names)
{
  Eigen::VectorXd position;
  position.resize(static_cast<long>(joint_state.position.size()));
  int i = 0;
  for (const auto& joint_name : joint_names)
  {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    assert(it != joint_state.name.end());
    size_t index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));
    position[i] = joint_state.position[index];
    ++i;
  }

  return position;
}

/**
 * @brief Convert a cartesian pose to cartesian waypoint.
 * @param pose The cartesian pose
 * @param change_base A tranformation applied to the pose = change_base * pose
 * @return WaypointPtr
 */
inline tesseract_motion_planners::Waypoint::Ptr
toWaypoint(const geometry_msgs::Pose& pose, Eigen::Isometry3d change_base = Eigen::Isometry3d::Identity())
{
  tesseract_motion_planners::CartesianWaypoint::Ptr waypoint =
      std::make_shared<tesseract_motion_planners::CartesianWaypoint>();
  Eigen::Isometry3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  waypoint->cartesian_position_ = change_base * pose_eigen;
  return std::move(waypoint);
}

/**
 * @brief Convert a vector of cartesian poses to vector of cartesian waypoints
 * @param poses The vector of cartesian poses
 * @param change_base A tranformation applied to the pose = change_base * pose
 * @return std::vector<WaypointPtr>
 */
inline std::vector<tesseract_motion_planners::Waypoint::Ptr>
toWaypoint(const std::vector<geometry_msgs::Pose>& poses, Eigen::Isometry3d change_base = Eigen::Isometry3d::Identity())
{
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  waypoints.reserve(poses.size());
  for (const auto& pose : poses)
    waypoints.push_back(toWaypoint(pose, change_base));

  return waypoints;
}

/**
 * @brief Convert a list of vector of cartesian poses to list of vector of cartesian waypoints
 * @param pose_arrays The list of vector of cartesian poses
 * @param change_base A tranformation applied to the pose = change_base * pose
 * @return std::vector<std::vector<WaypointPtr>>
 */
inline std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>>
toWaypoint(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
           Eigen::Isometry3d change_base = Eigen::Isometry3d::Identity())
{
  std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>> paths;
  paths.reserve(pose_arrays.size());
  for (const auto& pose_array : pose_arrays)
    paths.push_back(toWaypoint(pose_array.poses, change_base));

  return paths;
}

/**
 * @brief Convert a vector of double to joint waypoint
 * @param pose The joint positions
 * @return WaypointPtr
 */
inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const std::vector<double>& pose)
{
  tesseract_motion_planners::JointWaypoint::Ptr waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>();
  waypoint->joint_positions_ = toEigen(pose);
  return std::move(waypoint);
}

/**
 * @brief Convert a joint_state type to joint waypoint
 * @param joint_state The JointState to be converted
 * @param joint_names This is the desired order of the joints
 * @return WaypointPtr
 */
inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const sensor_msgs::JointState& joint_state,
                                                           const std::vector<std::string>& joint_names)
{
  tesseract_motion_planners::JointWaypoint::Ptr waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>();
  waypoint->joint_positions_ = toEigen(joint_state, joint_names);
  return std::move(waypoint);
}

/**
 * @brief Convert a vector of waypoints into a pose array
 * @param waypoints A vector of waypoints
 * @return Pose Array
 */
geometry_msgs::PoseArray toPoseArray(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints)
{
  geometry_msgs::PoseArray pose_array;
  for (const auto& wp : waypoints)
  {
    if (wp->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT)
    {
      geometry_msgs::Pose pose;
      const tesseract_motion_planners::CartesianWaypoint::Ptr& cwp =
          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(wp);
      tf::poseEigenToMsg(cwp->cartesian_position_, pose);
      pose_array.poses.push_back(pose);
    }
    else
    {
      ROS_ERROR("toPoseArray only support Cartesian Waypoints at this time.");
    }
  }

  return pose_array;
}

/**
 * @brief Convert a process definition into a single pose array
 * @param process_definition A process definition
 * @return Pose Array
 */
geometry_msgs::PoseArray toPoseArray(const tesseract_process_planners::ProcessDefinition& process_definition)
{
  geometry_msgs::PoseArray full_path;
  for (size_t i = 0; i < process_definition.segments.size(); ++i)
  {
    geometry_msgs::PoseArray poses = toPoseArray(process_definition.segments[i].approach);
    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

    poses = toPoseArray(process_definition.segments[i].process);
    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

    poses = toPoseArray(process_definition.segments[i].departure);
    full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());

    if (i < process_definition.transitions.size())
    {
      poses = toPoseArray(process_definition.transitions[i].transition_from_end);
      full_path.poses.insert(full_path.poses.end(), poses.poses.begin(), poses.poses.end());
    }
  }

  return full_path;
}

/**
 * @brief Convert a Tesseract Trajectory to Process Plan Path
 * @param trajectory Tesseract Trajectory
 * @param joint_names Joint names corresponding to the tesseract trajectory
 * @return A process plan path
 */
inline tesseract_msgs::ProcessPlanPath toProcessPlanPath(const tesseract_common::TrajArray& trajectory, const std::vector<std::string>& joint_names)
{
  tesseract_msgs::ProcessPlanPath path;
  if (trajectory.size() != 0)
    tesseract_rosutils::toMsg(path.trajectory, joint_names, trajectory);
  return path;
}

/**
 * @brief Convert a process plan segment to a process plan segment message
 * @param segment_results The process segment plan
 * @param joint_names Joint names corresponding to the tesseract trajectory
 * @return Process Segment Message
 */
inline tesseract_msgs::ProcessPlanSegment toProcessPlanSegement(const tesseract_process_planners::ProcessSegmentPlan& process_plan_segment, const std::vector<std::string>& joint_names)
{
  tesseract_msgs::ProcessPlanSegment process_segment;
  process_segment.approach = toProcessPlanPath(process_plan_segment.approach, joint_names);
  process_segment.process = toProcessPlanPath(process_plan_segment.process, joint_names);
  process_segment.departure = toProcessPlanPath(process_plan_segment.departure, joint_names);
  return process_segment;
}

/**
 * @brief Convert a Process Plan to a single Joint Trajector for visualization
 * @param process_plan Process Plan
 * @return Joint Trajectory
 */
trajectory_msgs::JointTrajectory toJointTrajectory(const tesseract_msgs::ProcessPlan& process_plan)
{
  // Initialize with path from home
  double t = 0;
  trajectory_msgs::JointTrajectory trajectory = process_plan.from_start.trajectory;
  if (!trajectory.points.empty())
    t = trajectory.points.back().time_from_start.toSec();
  else
  {
    trajectory.joint_names = process_plan.segments[0].process.trajectory.joint_names;
    trajectory.header = process_plan.segments[0].process.trajectory.header;
  }

  // Append process segments and transitions
  for (size_t i = 0; i < process_plan.segments.size(); ++i)
  {
    for (const auto& point : process_plan.segments[i].approach.trajectory.points)
    {
      trajectory.points.push_back(point);
      trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
    }
    t = trajectory.points.back().time_from_start.toSec();

    for (const auto& point : process_plan.segments[i].process.trajectory.points)
    {
      trajectory.points.push_back(point);
      trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
    }
    t = trajectory.points.back().time_from_start.toSec();

    for (const auto& point : process_plan.segments[i].departure.trajectory.points)
    {
      trajectory.points.push_back(point);
      trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
    }
    t = trajectory.points.back().time_from_start.toSec();

    if (i < (process_plan.segments.size() - 1))
    {
      for (const auto& point : process_plan.transitions[i].from_end.trajectory.points)
      {
        trajectory.points.push_back(point);
        trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
      }
      t = trajectory.points.back().time_from_start.toSec();
    }
  }

  // Append path to home
  for (const auto& point : process_plan.to_end.trajectory.points)
  {
    trajectory.points.push_back(point);
    trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }

  return trajectory;
}

/**
 * @brief Convert a joint trajector to csv formate and write to file
 * @param joint_trajectory Joint trajectory to be writen to file
 * @param file_path The location to save the file
 * @return true if successful
 */
bool toCSVFile(const trajectory_msgs::JointTrajectory& joint_trajectory, const std::string& file_path)
{
  std::ofstream myfile;
  myfile.open(file_path);

  // Write Joint names as header
  std::copy(joint_trajectory.joint_names.begin(),
            joint_trajectory.joint_names.end(),
            std::ostream_iterator<std::string>(myfile, ","));
  myfile << ",\n";
  for (const auto& point : joint_trajectory.points)
  {
    std::copy(point.positions.begin(), point.positions.end(), std::ostream_iterator<double>(myfile, ","));
    myfile << "," + std::to_string(point.time_from_start.toSec()) + ",\n";
  }
  myfile.close();
  return true;
}
}  // namespace tesseact_rosutils
#endif  // TESSERACT_ROSUTILS_CONVERSIONS_H
