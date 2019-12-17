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
#include <tesseract_rosutils/utils.h>

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
 * @brief Converts JointState position to Eigen vector in the order provided by joint_names
 * @param joint_state The JointState
 * @param joint_names The vector joint names used to order the output
 * @return Eigen::VectorXd in the same order as joint_names
 */
inline Eigen::VectorXd toEigen(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names)
{
  Eigen::VectorXd position;
  position.resize(static_cast<long>(joint_names.size()));
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
toWaypoint(const geometry_msgs::Pose& pose, const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity())
{
  Eigen::Isometry3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  return std::make_shared<tesseract_motion_planners::CartesianWaypoint>(change_base * pose_eigen);
}

/**
 * @brief Convert a vector of cartesian poses to vector of cartesian waypoints
 * @param poses The vector of cartesian poses
 * @param change_base A tranformation applied to the pose = change_base * pose
 * @return std::vector<WaypointPtr>
 */
inline std::vector<tesseract_motion_planners::Waypoint::Ptr>
toWaypoint(const std::vector<geometry_msgs::Pose>& poses,
           const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity())
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
           const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity())
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
inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const std::vector<double>& pose,
                                                           const std::vector<std::string>& names)
{
  return std::make_shared<tesseract_motion_planners::JointWaypoint>(toEigen(pose), names);
}

inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const sensor_msgs::JointState& joint_state)
{
  assert(joint_state.name.size() == joint_state.position.size());
  std::vector<std::string> joint_names = joint_state.name;
  Eigen::VectorXd joint_positions(joint_state.position.size());
  for (long i = 0; i < static_cast<long>(joint_state.position.size()); ++i)
    joint_positions[i] = joint_state.position[static_cast<size_t>(i)];

  return std::make_shared<tesseract_motion_planners::JointWaypoint>(joint_positions, joint_names);
}

/**
 * @brief Convert a Tesseract Trajectory to Process Plan Path
 * @param trajectory Tesseract Trajectory
 * @param joint_names Joint names corresponding to the tesseract trajectory
 * @return A process plan path
 */
inline tesseract_msgs::ProcessPlanPath toProcessPlanPath(const tesseract_common::JointTrajectory& joint_trajectory)
{
  tesseract_msgs::ProcessPlanPath path;
  if (joint_trajectory.trajectory.size() != 0)
    tesseract_rosutils::toMsg(path.trajectory, joint_trajectory);
  return path;
}

/**
 * @brief Convert a process plan segment to a process plan segment message
 * @param segment_results The process segment plan
 * @param joint_names Joint names corresponding to the tesseract trajectory
 * @return Process Segment Message
 */
inline tesseract_msgs::ProcessPlanSegment
toProcessPlanSegement(const tesseract_process_planners::ProcessSegmentPlan& process_plan_segment)
{
  tesseract_msgs::ProcessPlanSegment process_segment;
  process_segment.approach = toProcessPlanPath(process_plan_segment.approach);
  process_segment.process = toProcessPlanPath(process_plan_segment.process);
  process_segment.departure = toProcessPlanPath(process_plan_segment.departure);
  return process_segment;
}

/**
 * @brief Append a process segment to an existing joint_trajectory
 * @param joint_trajectory Trajectory to add the process segment
 * @param process_plan_segment Process plan segment
 * @return
 */
inline bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                              const tesseract_msgs::ProcessPlanSegment& process_plan_segment)
{
  double t = 0;
  if (!joint_trajectory.points.empty())
    t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.approach.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.process.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.departure.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }

  return true;
}

/**
 * @brief Convert a Process Plan to a single Joint Trajectory for visualization
 * This does not clear the trajectory passed in it appends.
 * @param joint_trajectory Joint Trajectory Message
 * @param process_plan Process Plan
 * @return True if successful, otherwise false
 */
inline bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                              const tesseract_msgs::ProcessPlan& process_plan)
{
  double t = 0;
  if (!joint_trajectory.points.empty())
    t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan.from_start.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  if (!process_plan.from_start.trajectory.points.empty())
  {
    joint_trajectory.joint_names = process_plan.segments[0].process.trajectory.joint_names;
    joint_trajectory.header = process_plan.segments[0].process.trajectory.header;
  }

  // Append process segments and transitions
  for (size_t i = 0; i < process_plan.segments.size(); ++i)
  {
    toJointTrajectory(joint_trajectory, process_plan.segments[i]);
    t = joint_trajectory.points.back().time_from_start.toSec();

    if (i < (process_plan.segments.size() - 1))
    {
      for (const auto& point : process_plan.transitions[i].from_end.trajectory.points)
      {
        joint_trajectory.points.push_back(point);
        joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
      }
      t = joint_trajectory.points.back().time_from_start.toSec();
    }
  }

  // Append path to home
  for (const auto& point : process_plan.to_end.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }

  return true;
}

/**
 * @brief Convert a joint trajector to csv formate and write to file
 * @param joint_trajectory Joint trajectory to be writen to file
 * @param file_path The location to save the file
 * @return true if successful
 */
inline bool toCSVFile(const trajectory_msgs::JointTrajectory& joint_trajectory, const std::string& file_path)
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
}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_CONVERSIONS_H
