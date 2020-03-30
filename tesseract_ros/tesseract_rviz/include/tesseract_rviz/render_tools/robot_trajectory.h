/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Based on MoveIt code authored by: Ioan Sucan, Adam Leeper */

#ifndef TESSERACT_RVIZ_ROBOT_TRAJECTORY_H
#define TESSERACT_RVIZ_ROBOT_TRAJECTORY_H

#include <deque>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

namespace tesseract_rviz
{
class RobotTrajectory
{
public:
  using Ptr = std::shared_ptr<RobotTrajectory>;
  using ConstPtr = std::shared_ptr<const RobotTrajectory>;

  RobotTrajectory(const trajectory_msgs::JointTrajectory& trajectory) : trajectory_(trajectory)
  {
    ros::Time last_time_stamp = trajectory_.header.stamp;
    ros::Time this_time_stamp = last_time_stamp;

    for (std::size_t i = 0; i < trajectory_.points.size(); ++i)
    {
      this_time_stamp = trajectory_.header.stamp + trajectory_.points[i].time_from_start;

      waypoints_.push_back(trajectory_.points[i]);
      duration_from_previous_.push_back((this_time_stamp - last_time_stamp).toSec());

      last_time_stamp = this_time_stamp;
    }
  }

  void findWayPointIndicesForDurationAfterStart(const double& duration, int& before, int& after, double& blend) const
  {
    if (duration < 0.0)
    {
      before = 0;
      after = 0;
      blend = 0;
      return;
    }

    // Find indicies
    std::size_t index = 0, num_points = waypoints_.size();
    double running_duration = 0.0;
    for (; index < num_points; ++index)
    {
      running_duration += duration_from_previous_[index];
      if (running_duration >= duration)
        break;
    }
    before = std::max<int>(index - 1, 0);
    after = std::min<int>(index, num_points - 1);

    // Compute duration blend
    double before_time = running_duration - duration_from_previous_[index];
    if (after == before)
    {
      blend = 1.0;
    }
    else
    {
      blend = (duration - before_time) / duration_from_previous_[index];
    }
  }

  bool getStateAtDurationFromStart(const double request_duration, sensor_msgs::JointState& output_state) const
  {
    // If there are no waypoints we can't do anything
    if (waypoints_.empty())
      return false;

    int before = 0;
    int after = 0;
    double blend = 1.0;
    findWayPointIndicesForDurationAfterStart(request_duration, before, after, blend);

    //    ROS_INFO("Interpolating %.3f of the way between index %d and %d.", blend, before, after);

    output_state = interpolate(waypoints_[before], waypoints_[after], blend);

    return true;
  }

  double getWayPointDurationFromStart(std::size_t index) const
  {
    if (duration_from_previous_.empty())
      return 0.0;
    if (index >= duration_from_previous_.size())
      index = duration_from_previous_.size() - 1;

    double time = 0.0;
    for (std::size_t i = 0; i <= index; ++i)
      time += duration_from_previous_[i];
    return time;
  }

  std::size_t getWayPointCount() const { return waypoints_.size(); }

private:
  sensor_msgs::JointState interpolate(const trajectory_msgs::JointTrajectoryPoint& start,
                                      const trajectory_msgs::JointTrajectoryPoint& end,
                                      const double t) const
  {
    sensor_msgs::JointState out;
    out.name = trajectory_.joint_names;
    out.position.resize(out.name.size());

    for (std::size_t i = 0; i < out.name.size(); ++i)
    {
      out.position[i] = start.positions[i] + (end.positions[i] - start.positions[i]) * t;
    }

    return out;
  }

  trajectory_msgs::JointTrajectory trajectory_;

  std::deque<trajectory_msgs::JointTrajectoryPoint> waypoints_;

  std::deque<double> duration_from_previous_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ROBOT_TRAJECTORY_H
